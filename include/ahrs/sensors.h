#ifndef AHRS_SENSORS_H
#define AHRS_SENSORS_H

#include "ahrs/numeric.h"
#include "ahrs/sensor_readout.h"

namespace ahrs {

/**
 * Interface for generic AHRS sensor. Implement it and inject to initialize
 * AHRS system.
 */
class Sensor {
   public:
    virtual sensor_readout read() = 0;
    virtual ~Sensor() = default;
};

class ImuCalibratedSensor {
   public: 
    explicit ImuCalibratedSensor(Sensor& imu_sensor) : imu_sensor{imu_sensor} {}
    sensor_readout read() const;
    virtual void calibrate_bias(const size_t num_of_samples = 100);
    virtual ~ImuCalibratedSensor() = default;

    sensor_readout offset_bias = {0.0, 0.0, 0.0};

   private:
    Sensor& imu_sensor;

    sensor_readout avg_n_readouts(const size_t n) const;
};

class GyroCalibratedSensor : public ImuCalibratedSensor {
   public:
    explicit GyroCalibratedSensor(Sensor& gyro) : ImuCalibratedSensor(gyro) {}
};

class AccelCalibratedSensor : public ImuCalibratedSensor {
   public:
    explicit AccelCalibratedSensor(Sensor& accel)
        : ImuCalibratedSensor(accel) {}
    void calibrate_bias(const size_t num_of_samples = 100) override;
};

class CompassCalibratedSensor {
   public:
    explicit CompassCalibratedSensor(Sensor& compass) : compass{compass} {}
    sensor_readout read() const;
    void calibrate_bias(const size_t num_of_samples = 1000);
    void calibrate_hard_iron(const size_t num_of_samples = 1000);
    void calibrate_soft_iron(const size_t num_of_samples = 1000);

    sensor_readout hard_iron_bias = {0.0, 0.0, 0.0};
    sensor_readout soft_iron_bias = {1.0, 1.0, 1.0};

   private:
    Sensor& compass;

    std::array<sensor_readout, 2>
    find_minmax_in_each_dimension(const size_t num_of_samples) const;
};

}  // namespace ahrs

#endif
