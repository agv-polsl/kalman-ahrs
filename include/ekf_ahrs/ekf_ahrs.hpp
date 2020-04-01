#include "ekf_ahrs/numeric.hpp"
#include <cmath>

namespace ekfn {

double calc_pitch(const double acc_x, double const acc_z) {
    return std::atan(acc_x / ((acc_x * acc_x) + (acc_z * acc_z)));
}

double calc_roll(const double acc_y, const double acc_z) {
    return std::atan(acc_y / ((acc_y * acc_y) + (acc_z * acc_z)));
}
}
