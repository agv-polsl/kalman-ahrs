#ifndef AHRS_KALMAN_H
#define AHRS_KALMAN_H

#include "ahrs/numeric.h"

namespace ahrs {

/**
 * This is a Kalman filter implementation.
 * Since this is based on control theory, please relate to academic sources
 * to understand this code. Names of the variables follow conventional math
 * symbols used for describing Kalman filter.
 */
class Kalman {
   public:
    Kalman(ahrs::array_2d<double, 4, 4> A, ahrs::array_2d<double, 4, 2> B,
           ahrs::array_2d<double, 2, 4> H)
        : A{A}, B{B}, H{H} {}
    ahrs::array_2d<double, 4, 1> update(
        ahrs::array_2d<double, 2, 1> input,
        ahrs::array_2d<double, 2, 1> measurement);
    void set_P_diagonal(const double val) noexcept;
    void set_Q_diagonal(const double val) noexcept;
    void set_R_diagonal(const double val) noexcept;

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
