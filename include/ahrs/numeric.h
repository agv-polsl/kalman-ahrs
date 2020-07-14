#ifndef AHRS_NUMERIC_H
#define AHRS_NUMERIC_H

#include <algorithm>
#include <array>
#include <cstddef>
#include <functional>
#include <ostream>

namespace ahrs {

template <typename T, size_t N, size_t M>
using array_2d = std::array<std::array<T, M>, N>;

template <typename T, size_t N, size_t M>
constexpr array_2d<T, N, M> zeros() {
    array_2d<T, N, M> ret;
    for (auto& row : ret) {
        std::fill(row.begin(), row.end(), static_cast<T>(0));
    }
    return ret;
}

template <typename T, size_t N>
constexpr array_2d<T, N, N> eye(T val = static_cast<T>(1)) {
    auto ret = zeros<T, N, N>();
    for (size_t i = 0; i < N; i++) {
        ret[i][i] = val;
    }
    return ret;
}

template <typename BinaryOperation, typename T, size_t N, size_t M>
array_2d<T, N, M> element_wise(const array_2d<T, N, M>& lhs,
                               const array_2d<T, N, M>& rhs,
                               BinaryOperation operation) noexcept {
    array_2d<T, N, M> ret;
    for (size_t i = 0; i < N; i++) {
        for (size_t j = 0; j < M; j++) {
            ret[i][j] = std::invoke(operation, lhs[i][j], rhs[i][j]);
        }
    }
    return ret;
}

template <typename T, size_t N, size_t M>
array_2d<T, N, M> operator+(const array_2d<T, N, M>& lhs,
                            const array_2d<T, N, M>& rhs) noexcept {
    return ahrs::element_wise(lhs, rhs, std::plus<T>());
}

template <typename T, size_t N, size_t M>
array_2d<T, N, M> operator-(const array_2d<T, N, M>& lhs,
                            const array_2d<T, N, M>& rhs) noexcept {
    return ahrs::element_wise(lhs, rhs, std::minus<T>());
}

template <typename T, size_t Nl, size_t Ml, size_t Nr, size_t Mr>
array_2d<T, Nl, Mr> operator*(const array_2d<T, Nl, Ml>& lhs,
                              const array_2d<T, Nr, Mr>& rhs) noexcept {
    static_assert(Ml == Nr, "Can not multiply matrices of given size");
    auto ret = zeros<T, Nl, Mr>();

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
array_2d<T, M, N> transpose(const array_2d<T, N, M>& arr) noexcept {
    array_2d<T, M, N> ret;
    for (size_t i = 0; i < M; i++) {
        for (size_t j = 0; j < N; j++) {
            ret[i][j] = arr[j][i];
        }
    }
    return ret;
}

template <typename T, size_t N>
array_2d<T, N, 2 * N> make_extended(const array_2d<T, N, N>& arr) noexcept {
    auto ret = zeros<T, N, 2 * N>();
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

template <typename T, size_t N>
array_2d<T, N, 2 * N>& reduce_to_echelon(array_2d<T, N, 2 * N>& arr) noexcept {
    for (size_t i = N - 1; i > 0; i--) {
        if (arr[i - 1][0] < arr[i][0]) {
            std::swap(arr[i], arr[i - 1]);
        }
    }
    return arr;
}

template <typename T, size_t N>
array_2d<T, N, 2 * N>& reduce_to_diag(array_2d<T, N, 2 * N>& arr) noexcept {
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
    return arr;
}

template <typename T, size_t N>
array_2d<T, N, 2 * N>& reduce_to_unit(array_2d<T, N, 2 * N>& arr) noexcept {
    for (size_t i = 0; i < N; i++) {
        auto temp = arr[i][i];
        for (size_t j = 0; j < 2 * N; j++) {
            arr[i][j] = arr[i][j] / temp;
        }
    }
    return arr;
}

template <typename T, size_t N>
array_2d<T, N, N> extract_inv(const array_2d<T, N, 2 * N>& arr) {
    auto ret = zeros<T, N, N>();
    for (size_t i = 0; i < N; i++) {
        std::copy_n(arr[i].cbegin() + N, N, ret[i].begin());
    }
    return ret;
}

template <typename T, size_t N>
array_2d<T, N, N> inv(const array_2d<T, N, N>& arr) {
    auto extended = make_extended(arr);
    auto reduced = reduce_to_unit(reduce_to_diag(reduce_to_echelon(extended)));
    return extract_inv(reduced);
}

template <typename T, size_t N, size_t M>
std::ostream& operator<<(std::ostream& os, const array_2d<T, N, M>& arr) {
    for (const auto& row : arr) {
        for (const auto& el : row) {
            os << el << ' ';
        }
        os << '\n';
    }
    return os;
}

}  // namespace ahrs

#endif
