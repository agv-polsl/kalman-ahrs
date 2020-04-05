#pragma once

namespace ahrs {

struct sensor_readout {
    double x, y, z;
};

class Sensor {
   public:
    virtual sensor_readout read() = 0;
    virtual void calibrate_bias(const int readout_num = 100);
    virtual sensor_readout fix_bias(sensor_readout);
    virtual ~Sensor();

   protected:
    sensor_readout bias;
};

class Gyro : public Sensor {};

class Accelerometer : public Sensor {};

class Magnetometer : public Sensor {
   public:
    void calibrate_bias(const int readout_num = 100) override;
    sensor_readout fix_bias(sensor_readout) override;

   protected:
    sensor_readout soft_iron_bias;
};

}  // namespace ahrs
