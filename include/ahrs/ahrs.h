#ifndef AHRS_AHRS_H
#define AHRS_AHRS_H

#include <chrono>
#include <cmath>

#include "ahrs/kalman.h"
#include "ahrs/numeric.h"
#include "ahrs/sensors.h"

namespace ahrs {

class Ahrs {
   public:
    Ahrs(Sensor& gyro, Sensor& acc, Sensor& mag,
         const std::chrono::duration<double> dt);
    void calibrate_imu(const size_t num_of_samples = 100);
    void calibrate_mag(const size_t num_of_samples = 1000);
    void set_dt(const std::chrono::duration<double> new_dt) noexcept;
    sensor_readout update();
    sensor_readout update(const std::chrono::duration<double> dt);

   private:
    ahrs::array_2d<double, 2, 1> calc_euler_angles_rates(
        sensor_readout gyro_read) const;
    ahrs::array_2d<double, 2, 1> calc_estimate(
          sensor_readout acc_read) const;

    GyroCalibratedSensor gyro;
    AccelCalibratedSensor acc;
    CompassCalibratedSensor mag;
    Kalman kalman;
    sensor_readout state = {0, 0, 0};
};

}  // namespace ahrs

#endif
