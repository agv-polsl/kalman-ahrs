#include "ahrs/ahrs.h"

namespace ahrs {

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

sensor_readout Ahrs::update() {
    auto gr = gyro.read();
    auto ar = acc.read();
    auto mr = mag.read();

    auto pitch = calc_pitch(ar.x, ar.z);
    auto roll = calc_roll(ar.y, ar.z);

    auto res = kalman.update({{{gr.x}, {gr.y}}}, {{{pitch}, {roll}}});

    auto yaw = calc_yaw(res[0][0], res[0][2], mr.x, mr.y, mr.z);

    return {res[0][0], res[0][2], yaw};
}

}  // namespace ahrs
