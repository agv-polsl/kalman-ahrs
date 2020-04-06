#pragma once

namespace ahrs {

struct sensor_readout {
    double x, y, z;
};

class Sensor {
   public:
    virtual sensor_readout read() = 0;
    virtual ~Sensor();
};

class CalibratedSensor {
   public:
    virtual sensor_readout read() = 0;
    virtual void calibrate_bias(int num_of_samples) = 0;
    virtual ~CalibratedSensor();

   protected:
    sensor_readout avg_n_readouts(int n);
};

class ImuCalibratedSensor : public CalibratedSensor {
   public:
    ImuCalibratedSensor(Sensor& imu_sensor) : imu_sensor{imu_sensor} {}
    sensor_readout read() override;
    void calibrate_bias(int num_of_samples = 100) override;

   private:
    Sensor& imu_sensor;
    sensor_readout offset_bias = {0.0, 0.0, 0.0};
};

class CompassCalibratedSensor : public CalibratedSensor {
   public:
    CompassCalibratedSensor(Sensor& compass) : compass{compass} {}
    sensor_readout read() override;
    void calibrate_bias(int num_of_samples = 100) override;
    void calibrate_hard_iron(int num_of_samples = 1000);
    void calibrate_soft_iron(int num_of_samples = 1000);

   private:
    Sensor& compass;
    sensor_readout hard_iron_bias = {0.0, 0.0, 0.0};
    sensor_readout soft_iron_bias = {1.0, 1.0, 1.0};
};

}  // namespace ahrs
