#pragma once

namespace ahrs {

struct sensor_readout {
    double x, y, z;
};

class Sensor {
   public:
    virtual sensor_readout read() = 0;
    virtual void calibrate_offset() = 0;
    virtual double get_offset() = 0;
    virtual sensor_readout fix_offset_bias(sensor_readout) = 0;
};

class Gyro : public Sensor {
   public:
    void calibrate_offset() override;
    virtual double get_offset() override;
    virtual sensor_readout fix_offset_bias(sensor_readout) override;

   protected:
    double offset = 0.0;
};

class Accelerometer : public Sensor {
   public:
    void calibrate_offset() override;
    virtual double get_offset() override;
    virtual sensor_readout fix_offset_bias(sensor_readout) override;

   protected:
    double offset = 0.0;
};

class Magnetometer : public Sensor {
   public:
    void calibrate_offset() override;
    virtual double get_offset() override;
    virtual sensor_readout fix_offset_bias(sensor_readout) override;

   protected:
    double offset = 0.0;
};

}  // namespace ahrs
