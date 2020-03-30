#include <array>
#include <ostream>

namespace ekfn { /* Extended kalman filter numerics */

template <typename T, size_t N, size_t M>
using array_2d = std::array<std::array<T, M>, N>;

template <typename T, size_t Nl, size_t Ml, size_t Nr, size_t Mr>
array_2d<T, Nl, Mr> operator*(const array_2d<T, Nl, Ml>& lhs,
                              const array_2d<T, Nr, Mr>& rhs) {
    static_assert(Ml == Nr, "Can not multiply matrices of given size");
    array_2d<T, Nl, Mr> ret = {0};

    for (size_t i = 0; i < Nl; i++) {
        for (size_t j = 0; j < Ml; j++) {
            for (size_t k = 0; k < Mr; k++) {
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
array_2d<T, N, M> transpose(const array_2d<T, N, M>& arr) {
    array_2d<T, N, M> ret = arr;
    for (size_t i = 0; i < N; i++) {
        for (size_t j = 0; j < M; j++) {
            ret[i][j] = arr[j][i];
        }
    }
    return ret;
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
array_2d<T, N, M> gauss_swap(const array_2d<T, N, M>& arr) {
    static_assert(M == 2 * N,
                  "Extended matrix must have twice more columns than rows");
    array_2d<T, N, M> ret = arr;
    for (size_t i = N - 1; i > 0; i--) {
        if (ret[i - 1][0] < ret[i][0]) {
            auto tmp = ret[i];
            ret[i] = ret[i - 1];
            ret[i - 1] = tmp;
        }
    }
    return ret;
}

template <typename T, size_t N, size_t M>
array_2d<T, N, M> gauss_reduce(const array_2d<T, N, M>& arr) {
    static_assert(M == 2 * N,
                  "Extended matrix must have twice more columns than rows");
    array_2d<T, N, M> ret = arr;

    for (size_t i = 0; i < N; i++) {
        for (size_t j = 0; j < N; j++) {
            if (j != i) {
                auto tmp = ret[j][i] / ret[i][i];
                for (size_t k = 0; k < 2 * N; k++) {
                    ret[j][k] -= ret[i][k] * tmp;
                }
            }
        }
    }

    for (size_t i = 0; i < N; i++) {
        auto temp = ret[i][i];
        for (size_t j = 0; j < 2 * N; j++) {
            ret[i][j] = ret[i][j] / temp;
        }
    }
    return ret;
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
    array_2d<T, N, N> ret;
    auto extended = add_identity(arr);
    extended = gauss_swap(extended);
    extended = gauss_reduce(extended);
    ret = extract_inv(extended);
    return ret;
}

}  // namespace ekfn
