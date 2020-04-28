#include "ahrs/ahrs.h"

#include <cmath>

namespace ahrs {

double to_deg(double rad) { return rad * 180 / M_PI; }

Ahrs::Ahrs(Sensor& gyro, Sensor& acc, Sensor& mag, double dt)
    : gyro_{gyro},
      acc_{acc},
      mag_{mag},
      kalman{{{{1, -dt, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, -dt}, {0, 0, 0, 1}}},
             {{{dt, 0}, {0, 0}, {0, dt}, {0, 0}}},
             {{{1, 0, 0, 0}, {0, 0, 1, 0}}}} {
    calibrate();
}

void Ahrs::calibrate() {
    gyro_.calibrate_bias();
    acc_.calibrate_bias();
    mag_.calibrate_bias();
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

sensor_readout Ahrs::update(double dt) {
    set_dt(dt);
    return update();
}

}  // namespace ahrs
