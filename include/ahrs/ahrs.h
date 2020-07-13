#ifndef AHRS_H
#define AHRS_H

#include <cmath>

#include "ahrs/sensors.h"
#include "ahrs/kalman.h"
#include "ahrs/numeric.h"

namespace ahrs {

class Ahrs {
   public:
    Ahrs(Sensor& gyro, Sensor& acc, Sensor& mag, double dt);
    void calibrate_imu();
    void calibrate_mag(int num_of_samples = 1000);
    void set_dt(double new_dt);
    sensor_readout update();
    sensor_readout update(double dt);

   private:
    ahrs::array_2d<double, 2, 1> calc_euler_angles_rates(sensor_readout gyro);
    ahrs::array_2d<double, 2, 1> calc_estimate(sensor_readout acc);

    ImuCalibratedSensor gyro_;
    ImuCalibratedSensor acc_;
    CompassCalibratedSensor mag_;
    Kalman kalman;
    sensor_readout state = {0, 0, 0};
};

}  // namespace ahrs

#endif
