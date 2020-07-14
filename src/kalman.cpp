#include "ahrs/kalman.h"

namespace ahrs {

void Kalman::predict(const ahrs::array_2d<double, 2, 1>& u) noexcept {
    x = A * x + B * u;
    P = A * P * ahrs::transpose(A) + Q;
}

void Kalman::correct(const ahrs::array_2d<double, 2, 1>& z) {
    auto K = P * ahrs::transpose(H) * ahrs::inv(H * P * ahrs::transpose(H) + R);
    x = x + K * (z - H * x);
    P = P - K * H * P;
}

ahrs::array_2d<double, 4, 1> Kalman::update (
    ahrs::array_2d<double, 2, 1> input,
    ahrs::array_2d<double, 2, 1> measurement) {

    predict(input);
    correct(measurement);
    return x;
}

}  // namespace ahrs
