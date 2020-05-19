#ifndef SENSOR_READOUT_H
#define SENSOR_READOUT_H

#include <functional>
#include <ostream>

namespace ahrs {

struct sensor_readout {
    double x, y, z;

    std::ostream& operator<<(std::ostream& os) {
        os << x << ' ' << y << ' ' << z;
        return os;
    }

    template <typename T>
    bool operator==(const T rhs) {
        return element_wise(rhs, std::equal_to<double>());
    }

    template <typename T>
    bool operator!=(const T rhs) {
        return element_wise(rhs, std::not_equal_to<double>());
    }
    template <typename T>
    sensor_readout operator+(const T rhs) {
        return element_wise(rhs, std::plus<double>());
    }

    template <typename T>
    sensor_readout operator+=(T rhs) {
        *this = element_wise(rhs, std::plus<double>());
        return *this;
    }

    template <typename T>
    sensor_readout operator-(T rhs) {
        return element_wise(rhs, std::minus<double>());
    }

    template <typename T>
    sensor_readout operator-=(T rhs) {
        *this = element_wise(rhs, std::minus<double>());
        return *this;
    }
    template <typename T>
    sensor_readout operator*(T rhs) {
        return element_wise(rhs, std::multiplies<double>());
    }

    template <typename T>
    sensor_readout operator*=(T rhs) {
        *this = element_wise(rhs, std::multiplies<double>());
        return *this;
    }

    template <typename T>
    sensor_readout operator/(T rhs) {
        return element_wise(rhs, std::divides<double>());
    }

    template <typename T>
    sensor_readout operator/=(T rhs) {
        *this = element_wise(rhs, std::divides<double>());
        return *this;
    }

   private:
    template <typename BinaryOperation>
    sensor_readout element_wise(const sensor_readout rhs,
                                BinaryOperation operation) {
        return {std::invoke(operation, x, rhs.x),
                std::invoke(operation, y, rhs.y),
                std::invoke(operation, z, rhs.z)};
    }

    template <typename BinaryOperation>
    sensor_readout element_wise(const double rhs, BinaryOperation operation) {
        return {std::invoke(operation, x, rhs),
                std::invoke(operation, y, rhs),
                std::invoke(operation, z, rhs)};
    }
};

}  // namespace ahrs

#endif
