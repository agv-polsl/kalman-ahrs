#include <array>

namespace ekfn { /* Extended kalman filter numerics */

template <typename T, size_t N, size_t M>
using array_2d = std::array<std::array<T, M>, N>;

template <typename T, size_t Nl, size_t Ml, size_t Nr, size_t Mr>
array_2d<T, Nl, Mr> operator*(const array_2d<T, Nl, Ml>& lhs,
                              const array_2d<T, Nr, Mr>& rhs) {
    array_2d<T, Nl, Mr> ret = {0};
    return ret;
}

template <typename T, size_t N, size_t M>
std::ostream& operator<<(std::ostream& stream, const array_2d<T, N, M>& arr) {
    return stream;
}

template <typename T, size_t N, size_t M>
array_2d<T, N, M> transpose(const array_2d<T, N, M>& arr) {
    array_2d<T, N, M> ret = arr;
    return ret;
}

template <typename T, size_t N>
array_2d<T, N, 2 * N> add_identity(const array_2d<T, N, N>& arr) {
    array_2d<T, N, 2 * N> ret = {0};
    return ret;
}

template <typename T, size_t N, size_t M>
array_2d<T, N, M> gauss_swap(const array_2d<T, N, M>& arr) {
    array_2d<T, N, M> ret = arr;
    return ret;
}

template <typename T, size_t N, size_t M>
array_2d<T, N, M> gauss_reduce(const array_2d<T, N, M>& arr) {
    array_2d<T, N, M> ret = arr;
    return ret;
}

template <typename T, size_t N, size_t M>
array_2d<T, N, N> extract_inv(const array_2d<T, N, M>& arr) {
    array_2d<T, N, N> ret;
    return ret;
}

template <typename T, size_t N, size_t M>
array_2d<T, N, M> inv(const array_2d<T, N, M>& arr) {
    array_2d<T, N, M> ret;
    return ret;
}

}  // namespace ekfn
