#ifndef AHRS_H
#define AHRS_H

#include <cmath>

#include "ahrs/ahrs_sensors.h"
#include "ahrs/kalman.h"
#include "ahrs/numeric.h"

namespace ahrs {

class Ahrs {
   public:
    Ahrs(Sensor& gyro, Sensor& acc, Sensor& mag, double dt);
    void calibrate();
    void set_dt(double new_dt);
    sensor_readout update();
    sensor_readout update(double dt);

    static double calc_roll(const sensor_readout acc) {
        return std::atan2(acc.y, sqrt(pow(acc.x, 2) + pow(acc.z, 2)));
    }

    static double calc_pitch(const sensor_readout acc) {
        return std::atan2(-acc.x, sqrt(pow(acc.y, 2) + pow(acc.z, 2)));
    }

    static double calc_yaw(const double pitch, const double roll,
                           const sensor_readout mag) {
        auto horizon_plane_x = mag.x * cos(pitch) +
                               mag.y * sin(pitch) * sin(roll) +
                               mag.z * sin(pitch) + cos(roll);
        auto horizon_plane_y = mag.y * cos(roll) - mag.z * sin(roll);

        return std::atan2(-horizon_plane_y, horizon_plane_x);
    }

   private:
    ImuCalibratedSensor gyro;
    ImuCalibratedSensor acc;
    CompassCalibratedSensor mag;
    Kalman kalman;
};

}  // namespace ahrs

#endif
