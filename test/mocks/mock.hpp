#include <gmock/gmock.h>

#include "ahrs/ahrs_sensors.hpp"

class SensorMock : Sensor {
   public:
    MOCK_METHOD(sensor_readout, read, void);
};
