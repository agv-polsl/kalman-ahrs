#pragma once

#include "ahrs/numeric.h"

namespace ahrs {

class Kalman {
   public:
    Kalman(ahrs::array_2d<double, 4, 4> A, ahrs::array_2d<double, 4, 2> B,
           ahrs::array_2d<double, 2, 4> H)
        : A{A}, B{B}, H{H} {}

    auto update(ahrs::array_2d<double, 2, 1> input,
                ahrs::array_2d<double, 2, 1> measurement) {
        predict(input);
        correct(measurement);
        return x;
    }

   private:
    ahrs::array_2d<double, 4, 4> A;
    ahrs::array_2d<double, 4, 2> B;
    ahrs::array_2d<double, 2, 4> H;

    ahrs::array_2d<double, 4, 1> x = ahrs::zeros<double, 4, 1>();
    ahrs::array_2d<double, 4, 4> P = ahrs::zeros<double, 4, 4>();

    ahrs::array_2d<double, 4, 4> Q = ahrs::eye<double, 4>();
    ahrs::array_2d<double, 2, 2> R = ahrs::eye<double, 2>();

    void predict(const ahrs::array_2d<double, 2, 1>& u) {
        x = A * x + B * u;
        P = A * P * ahrs::transpose(A) + Q;
    }

    void correct(const ahrs::array_2d<double, 2, 1>& z) {
        auto K =
            P * ahrs::transpose(H) * ahrs::inv(H * P * ahrs::transpose(H) + R);
        x = x + K * (z - H * x);
        P = P - K * H * P;
    }
};

}  // namespace ahrs
