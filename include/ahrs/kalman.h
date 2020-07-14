#ifndef AHRS_KALMAN_H
#define AHRS_KALMAN_H

#include "ahrs/numeric.h"

namespace ahrs {

class Kalman {
   public:
    Kalman(ahrs::array_2d<double, 4, 4> A, ahrs::array_2d<double, 4, 2> B,
           ahrs::array_2d<double, 2, 4> H)
        : A{A}, B{B}, H{H} {}
    ahrs::array_2d<double, 4, 1> update(
        ahrs::array_2d<double, 2, 1> input,
        ahrs::array_2d<double, 2, 1> measurement);

    ahrs::array_2d<double, 4, 4> A;
    ahrs::array_2d<double, 4, 2> B;
    ahrs::array_2d<double, 2, 4> H;

   private:
    void correct(const ahrs::array_2d<double, 2, 1>& z);
    void predict(const ahrs::array_2d<double, 2, 1>& u) noexcept;

    ahrs::array_2d<double, 4, 1> x = ahrs::zeros<double, 4, 1>();

    ahrs::array_2d<double, 4, 4> P = ahrs::eye<double, 4>();
    ahrs::array_2d<double, 4, 4> Q = ahrs::eye<double, 4>();
    ahrs::array_2d<double, 2, 2> R = ahrs::eye<double, 2>();
};

}  // namespace ahrs

#endif
