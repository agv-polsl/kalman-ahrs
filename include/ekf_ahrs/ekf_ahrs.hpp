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

class Kalman {
   public:
    double dt;

    ekfn::array_2d<double, 4, 1> x = { 0 };

    ekfn::array_2d<double, 4, 4> A;
    ekfn::array_2d<double, 4, 2> B;
    ekfn::array_2d<double, 2, 4> H;

    ekfn::array_2d<double, 4, 4> P = { 0   };
    ekfn::array_2d<double, 4, 4> Q;
    ekfn::array_2d<double, 2, 2> R;

    void predict(ekfn::array_2d<double, 2, 1> u) {
        x = A * x + B * u;
        P = A * P * ekfn::transpose(A) + Q;
    }

    void correct(ekfn::array_2d<double, 2, 1> z) {
        auto K =
            P * ekfn::transpose(H) * ekfn::inv(H * P * ekfn::transpose(H) + R);
        x = x + K * (z - H * x);
        P = P - K * H * P;
    }

   public:
    Kalman(ekfn::array_2d<double, 4, 4> A, ekfn::array_2d<double, 4, 4> Q,
           ekfn::array_2d<double, 2, 2> R)
        : A{A}, Q{Q}, R{R} {}

    auto update(ekfn::array_2d<double, 2, 1> input, ekfn::array_2d<double, 2, 1> measurement) {
        predict(input);
        correct(measurement);
        return x;
    }
};

}  // namespace ekfn
