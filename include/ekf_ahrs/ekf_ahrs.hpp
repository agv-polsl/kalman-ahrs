#include <cmath>

#include "ekf_ahrs/numeric.hpp"

namespace ekfn {

double calc_pitch(const double acc_x, double const acc_z) {
    return std::atan(acc_x / ((acc_x * acc_x) + (acc_z * acc_z)));
}

double calc_roll(const double acc_y, const double acc_z) {
    return std::atan(acc_y / ((acc_y * acc_y) + (acc_z * acc_z)));
}

double calc_yaw(const double pitch, const double roll, const double mag_x,
                const double mag_y, const double mag_z) {
    /* TODO: This may be wrong due to the inconvenience in the reference
     * academic paper, for now it is left as is for the sake of continuing
     * development. This must be resolved in future.
     */
    auto horizon_plane_x = mag_x * cos(pitch) + mag_y * sin(pitch) * sin(roll) +
                           mag_z * sin(pitch) + cos(roll);
    auto horizon_plane_y = mag_y * cos(roll) + mag_z * sin(roll);

    return std::atan(-horizon_plane_y / horizon_plane_x);
}

}  // namespace ekfn
