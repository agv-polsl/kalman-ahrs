#ifndef AHRS_SENSOR_READOUT_H
#define AHRS_SENSOR_READOUT_H

#include <functional>
#include <ostream>

namespace ahrs {

struct sensor_readout {
    double x, y, z;

    inline bool operator==(const sensor_readout& rhs) const noexcept {
        return x == rhs.x && y == rhs.y && z == rhs.z;
    }

    inline bool operator!=(const sensor_readout& rhs) const noexcept {
        return x != rhs.x || y != rhs.y || z != rhs.z;
    }

    template <typename T>
    sensor_readout operator+(const T rhs) const noexcept {
        return element_wise(rhs, std::plus());
    }

    template <typename T>
    sensor_readout operator+=(T rhs) noexcept {
        *this = element_wise(rhs, std::plus());
        return *this;
    }

    template <typename T>
    sensor_readout operator-(T rhs) const noexcept {
        return element_wise(rhs, std::minus());
    }

    template <typename T>
    sensor_readout operator-=(T rhs) noexcept {
        *this = element_wise(rhs, std::minus());
        return *this;
    }
    template <typename T>
    sensor_readout operator*(T rhs) const noexcept {
        return element_wise(rhs, std::multiplies());
    }

    template <typename T>
    sensor_readout operator*=(T rhs) noexcept {
        *this = element_wise(rhs, std::multiplies());
        return *this;
    }

    template <typename T>
    sensor_readout operator/(T rhs) const noexcept {
        return element_wise(rhs, std::divides());
    }

    template <typename T>
    sensor_readout operator/=(T rhs) noexcept {
        *this = element_wise(rhs, std::divides());
        return *this;
    }

    template <typename BinaryOperation>
    sensor_readout element_wise(const sensor_readout rhs,
                                BinaryOperation operation) const noexcept {
        return {std::invoke(operation, x, rhs.x),
                std::invoke(operation, y, rhs.y),
                std::invoke(operation, z, rhs.z)};
    }

    template <typename BinaryOperation>
    sensor_readout element_wise(const double rhs,
                                BinaryOperation operation) const noexcept {
        return {std::invoke(operation, x, rhs), std::invoke(operation, y, rhs),
                std::invoke(operation, z, rhs)};
    }
};

inline std::ostream& operator<<(std::ostream& os,
                                const sensor_readout& sr) noexcept {
    os << sr.x << ' ' << sr.y << ' ' << sr.z;
    return os;
}

}  // namespace ahrs

#endif
