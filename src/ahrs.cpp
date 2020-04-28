#include "ahrs/ahrs.h"

#include <cmath>

namespace ahrs {

double to_deg(double rad) { return rad * 180 / M_PI; }

Ahrs::Ahrs(Sensor& gyro, Sensor& acc, Sensor& mag, double dt)
    : gyro{gyro},
      acc{acc},
      mag{mag},
      kalman{{{{1, -dt, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, -dt}, {0, 0, 0, 1}}},
             {{{dt, 0}, {0, 0}, {0, dt}, {0, 0}}},
             {{{1, 0, 0, 0}, {0, 0, 1, 0}}}} {
    calibrate();
}

void Ahrs::calibrate() {
    gyro.calibrate_bias();
    acc.calibrate_bias();
    mag.calibrate_bias();
}

void Ahrs::set_dt(double dt) {
    kalman.A[0][1] = -dt;
    kalman.A[2][3] = -dt;
    kalman.B[0][0] = dt;
    kalman.B[3][1] = dt;
}

sensor_readout Ahrs::update() {
    auto gr = gyro.read();
    auto ar = acc.read();
    auto mr = mag.read();

    auto roll = calc_roll(ar);
    auto pitch = calc_pitch(ar);

    auto res = kalman.update({{{gr.x}, {gr.y}}}, {{{roll}, {pitch}}});

    auto yaw = calc_yaw(res[0][0], res[0][2], mr);

    return {to_deg(res[0][0]), to_deg(res[0][2]), to_deg(yaw)};
}

sensor_readout Ahrs::update(double dt) {
    set_dt(dt);
    return update();
}

}  // namespace ahrs
