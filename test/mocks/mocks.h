#include <gmock/gmock.h>

#include "ahrs/ahrs_sensors.h"

class SensorMock : public ahrs::Sensor {
   public:
    MOCK_METHOD(ahrs::sensor_readout, read, ());
};
