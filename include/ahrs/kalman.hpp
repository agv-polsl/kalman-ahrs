#pragma once

#include "ahrs/numeric.hpp"

namespace ekfn {

class Kalman {
   public:
    Kalman(ekfn::array_2d<double, 4, 4> A, ekfn::array_2d<double, 4, 2> B,
           ekfn::array_2d<double, 2, 4> H, double dt)
        : A{A}, B{B}, H{H}, dt{dt} {}

    auto update(ekfn::array_2d<double, 2, 1> input,
                ekfn::array_2d<double, 2, 1> measurement) {
        predict(input);
        correct(measurement);
        return x;
    }

   private:
    double dt;

    ekfn::array_2d<double, 4, 4> A;
    ekfn::array_2d<double, 4, 2> B;
    ekfn::array_2d<double, 2, 4> H;

    ekfn::array_2d<double, 4, 1> x = {0};
    ekfn::array_2d<double, 4, 4> P = {0};

    ekfn::array_2d<double, 4, 4> Q = ekfn::eye<double, 4>();
    ekfn::array_2d<double, 2, 2> R = ekfn::eye<double, 2>();

    void predict(const ekfn::array_2d<double, 2, 1>& u) {
        x = A * x + B * u;
        P = A * P * ekfn::transpose(A) + Q;
    }

    void correct(const ekfn::array_2d<double, 2, 1>& z) {
        auto K =
            P * ekfn::transpose(H) * ekfn::inv(H * P * ekfn::transpose(H) + R);
        x = x + K * (z - H * x);
        P = P - K * H * P;
    }
};

}  // namespace ekfn
