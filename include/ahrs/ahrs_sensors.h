#ifndef AHRS_SENSORS_H
#define AHRS_SENSORS_H

#include "ahrs/numeric.h"
#include "ahrs/sensor_readout.h"

namespace ahrs {

class Sensor {
   public:
    virtual sensor_readout read() = 0;
    virtual ~Sensor() {}
};

class ImuCalibratedSensor {
   public:
    ImuCalibratedSensor(Sensor& imu_sensor) : imu_sensor{imu_sensor} {}
    sensor_readout read();
    void calibrate_bias(int num_of_samples = 100);

   private:
    Sensor& imu_sensor;
    sensor_readout offset_bias = {0.0, 0.0, 0.0};

    sensor_readout avg_n_readouts(int n);
};

class CompassCalibratedSensor {
   public:
    CompassCalibratedSensor(Sensor& compass) : compass{compass} {}
    sensor_readout read();
    void calibrate_bias(int num_of_samples = 1000);
    void calibrate_hard_iron(int num_of_samples = 1000);
    void calibrate_soft_iron(int num_of_samples = 1000);

   private:
    Sensor& compass;
    sensor_readout hard_iron_bias = {0.0, 0.0, 0.0};
    sensor_readout soft_iron_bias = {1.0, 1.0, 1.0};

    std::array<sensor_readout, 2>
    find_minmax_in_each_dimension(int num_of_samples);
    static sensor_readout update_min(sensor_readout newr, sensor_readout minr);
    static sensor_readout update_max(sensor_readout newr, sensor_readout maxr);
};

}  // namespace ahrs

#endif
