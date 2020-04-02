#include <algorithm>
#include <array>
#include <functional>
#include <iostream>
#include <ostream>

namespace ekfn { /* Extended kalman filter numerics */

template <typename T, size_t N, size_t M>
using array_2d = std::array<std::array<T, M>, N>;

template <typename BinaryOperation, typename T, size_t N, size_t M>
array_2d<T, N, M> element_wise(const array_2d<T, N, M>& lhs,
                               const array_2d<T, N, M>& rhs,
                               BinaryOperation operation) {
    array_2d<T, N, M> ret = {0};
    for (size_t i = 0; i < N; i++) {
        for (size_t j = 0; j < M; j++) {
            ret[i][j] = std::invoke(operation, lhs[i][j], rhs[i][j]);
        }
    }
    return ret;
}

template <typename T, size_t N, size_t M>
array_2d<T, N, M> operator+(const array_2d<T, N, M>& lhs,
                            const array_2d<T, N, M>& rhs) {
    return ekfn::element_wise(lhs, rhs, std::plus<T>());
}

template <typename T, size_t N, size_t M>
array_2d<T, N, M> operator-(const array_2d<T, N, M>& lhs,
                            const array_2d<T, N, M>& rhs) {
    return ekfn::element_wise(lhs, rhs, std::minus<T>());
}

template <typename T, size_t Nl, size_t Ml, size_t Nr, size_t Mr>
array_2d<T, Nl, Mr> operator*(const array_2d<T, Nl, Ml>& lhs,
                              const array_2d<T, Nr, Mr>& rhs) {
    static_assert(Ml == Nr, "Can not multiply matrices of given size");
    array_2d<T, Nl, Mr> ret = {0};

    for (size_t i = 0; i < Nl; i++) {
        for (size_t j = 0; j < Mr; j++) {
            for (size_t k = 0; k < Ml; k++) {
                ret[i][j] += lhs[i][k] * rhs[k][j];
            }
        }
    }
    return ret;
}

template <typename T, size_t N, size_t M>
std::ostream& operator<<(std::ostream& os, const array_2d<T, N, M> arr) {
    for (auto row : arr) {
        for (auto el : row) {
            os << el << ' ';
        }
        os << '\n';
    }
    return os;
}

template <typename T, size_t N, size_t M>
array_2d<T, M, N> transpose(const array_2d<T, N, M>& arr) {
    array_2d<T, M, N> res = {0};
    for (size_t i = 0; i < M; i++) {
        for (size_t j = 0; j < N; j++) {
            res[i][j] = arr[j][i];
        }
    }
    return res;
}

/* Below are function to perform matrix inversion using Gauss-Jordan
 * elimination method.
 */

template <typename T, size_t N>
array_2d<T, N, 2 * N> add_identity(const array_2d<T, N, N>& arr) {
    array_2d<T, N, 2 * N> ret = {0};
    for (size_t i = 0; i < N; i++) {
        for (size_t j = 0; j < 2 * N; j++) {
            if (j < N) {
                /* Copy given array to return */
                ret[i][j] = arr[i][j];
            } else if (j == (i + N)) {
                /* Add identity matrix on the left of ret matix */
                ret[i][j] = 1;
            }
        }
    }
    return ret;
}

template <typename T, size_t N, size_t M>
array_2d<T, N, M>& gauss_swap(array_2d<T, N, M>& arr) {
    static_assert(M == 2 * N,
                  "Extended matrix must have twice more columns than rows");
    for (size_t i = N - 1; i > 0; i--) {
        if (arr[i - 1][0] < arr[i][0]) {
            auto tmp = arr[i];
            arr[i] = arr[i - 1];
            arr[i - 1] = tmp;
        }
    }
    return arr;
}

template <typename T, size_t N, size_t M>
array_2d<T, N, M>& gauss_reduce(array_2d<T, N, M>& arr) {
    static_assert(M == 2 * N,
                  "Extended matrix must have twice more columns than rows");

    for (size_t i = 0; i < N; i++) {
        for (size_t j = 0; j < N; j++) {
            if (j != i) {
                auto tmp = arr[j][i] / arr[i][i];
                for (size_t k = 0; k < 2 * N; k++) {
                    arr[j][k] -= arr[i][k] * tmp;
                }
            }
        }
    }

    for (size_t i = 0; i < N; i++) {
        auto temp = arr[i][i];
        for (size_t j = 0; j < 2 * N; j++) {
            arr[i][j] = arr[i][j] / temp;
        }
    }
    return arr;
}

template <typename T, size_t N, size_t M>
array_2d<T, N, N> extract_inv(const array_2d<T, N, M>& arr) {
    static_assert(M == 2 * N,
                  "Extended matrix must have twice more columns than rows");
    array_2d<T, N, N> ret;
    /* Copy right half of the extended matrix */
    for (int i = 1; i < N + 1; i++) {
        std::copy_n(arr[i].rend(), N, ret[i].rend());
    }
    return ret;
}

template <typename T, size_t N>
array_2d<T, N, N> inv(const array_2d<T, N, N>& arr) {
    auto extended = add_identity(arr);
    return extract_inv(gauss_reduce(gauss_swap(extended)));
}

}  // namespace ekfn
