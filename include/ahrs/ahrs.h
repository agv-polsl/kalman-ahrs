#ifndef AHRS_AHRS_H
#define AHRS_AHRS_H

#include <chrono>
#include <cmath>

#include "ahrs/kalman.h"
#include "ahrs/numeric.h"
#include "ahrs/sensors.h"

namespace ahrs {

/**
 * An AHRS system for gyro, accelerometer and magnetometer combo.
 * Initialized by injecting implemented Sensor interface for each sensor.
 * Features built in calibration mechanism. Allows using fixed or non constant
 * sampling period of dt.
 */
class Ahrs {
   public:
    /**
     * Initialize AHRS by injecting implemented Sensor interface for each
     * sensor and an initial sampling period.
     *
     * Notice that readouts from sensors should be in specific units.
     * Acceleration should be in meters per second squared,
     * angular velocity (from gyro) should be in degrees per second
     * and magnetic induction should be in gauss.
     */
    Ahrs(Sensor& gyro, Sensor& acc, Sensor& mag,
         const std::chrono::duration<double> dt);
    /**
     * Calibrate IMU sensors, note that during this process the AHRS system
     * should stand still on an even surface.
     */
    void calibrate_imu(const size_t num_of_samples = 100);
    /**
     * Calibrate magnetometer, note that during this process the ARHS system
     * should be rotated around as much as possible.
     */
    void calibrate_mag(const size_t num_of_samples = 1000);
    /**
     * Set new sampling period value that is used during calculations.
     */
    void set_dt(const std::chrono::duration<double> new_dt) noexcept;
    /**
     * Set diagonal of P matrix (estimate uncertanity).
     */
    void set_P_diagonal(const double val) noexcept;
    /**
     * Set diagonal of Q matrix (process noise uncertanity).
     */
    void set_Q_diagonal(const double val) noexcept;
    /**
     * Set diagonal of R matrix (mreasurement uncertanity).
     */
    void set_R_diagonal(const double val) noexcept;
    /**
     * Update the system cycle and get current roll, pitch, yaw angles
     * in degrees.
     */
    sensor_readout update();
    /**
     * Update the system cycle with new sampling period
     * and get current roll, pitch, yaw angles in degrees.
     */
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
