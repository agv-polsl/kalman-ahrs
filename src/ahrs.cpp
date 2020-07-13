#include "ahrs/ahrs.h"

#include <cmath>

namespace ahrs {

/* Helper functions */
static double to_deg(double rad) { return rad * 180 / M_PI; }

static double calc_roll(const sensor_readout acc) {
    return std::atan2(acc.y, sqrt(pow(acc.x, 2) + pow(acc.z, 2)));
}

static double calc_pitch(const sensor_readout acc) {
    return std::atan2(-acc.x, sqrt(pow(acc.y, 2) + pow(acc.z, 2)));
}

static double calc_yaw(const double roll, const double pitch,
                       const sensor_readout mag) {
    auto horizon_plane_x = mag.x * cos(pitch) + mag.y * sin(pitch) * sin(roll) +
                           mag.z * sin(pitch) * cos(roll);
    auto horizon_plane_y = mag.y * cos(roll) - mag.z * sin(roll);

    return std::atan2(-horizon_plane_y, horizon_plane_x);
}

/* Class implementation */
Ahrs::Ahrs(Sensor& gyro, Sensor& acc, Sensor& mag, double dt)
    : gyro_{gyro},
      acc_{acc},
      mag_{mag},
      kalman{{{{1, -dt, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, -dt}, {0, 0, 0, 1}}},
             {{{dt, 0}, {0, 0}, {0, dt}, {0, 0}}},
             {{{1, 0, 0, 0}, {0, 0, 1, 0}}}} {}

void Ahrs::calibrate_imu() {
    gyro_.calibrate_bias();
    acc_.calibrate_bias();
}

void Ahrs::calibrate_mag(int num_of_samples) {
    mag_.calibrate_bias(num_of_samples);
}

void Ahrs::set_dt(double dt) {
    kalman.A[0][1] = -dt;
    kalman.A[2][3] = -dt;
    kalman.B[0][0] = dt;
    kalman.B[2][1] = dt;
}

sensor_readout Ahrs::update() {
    auto gr = gyro_.read();
    auto ar = acc_.read();
    auto mr = mag_.read();

    auto system_input_vector = calc_euler_angles_rates(gr);
    auto estimate_vector = calc_estimate(ar);

    auto res = kalman.update(system_input_vector, estimate_vector);
    auto yaw = calc_yaw(res[0][0], res[0][2], mr);

    state = {to_deg(res[0][0]), to_deg(res[0][2]), to_deg(yaw)};

    return state;
}

ahrs::array_2d<double, 2, 1> Ahrs::calc_euler_angles_rates(
    sensor_readout gyro) {
    auto roll_rate = gyro.x + sin(state.x) * tan(state.y) * gyro.y +
                     cos(state.x) * tan(state.y) * gyro.z;
    auto pitch_rate = cos(state.x) * gyro.y - sin(state.x) * gyro.z;

    return {{{roll_rate}, {pitch_rate}}};
}

ahrs::array_2d<double, 2, 1> Ahrs::calc_estimate(sensor_readout acc) {
    auto roll_estimate = calc_roll(acc);
    auto pitch_estitmate = calc_pitch(acc);

    return {{{roll_estimate}, {pitch_estitmate}}};
}

sensor_readout Ahrs::update(double dt) {
    set_dt(dt);
    return update();
}

}  // namespace ahrs
