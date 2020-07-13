#include "ahrs/ahrs.h"

#include <gmock/gmock-nice-strict.h>
#include <gtest/gtest.h>

#include "mocks.h"

using namespace ::testing;
using namespace ahrs;

class AhrsTest : public Test {
   public:
    NiceMock<SensorMock> gyro;
    NiceMock<SensorMock> acc;
    NiceMock<SensorMock> mag;
    double dt = 0.01;

    ahrs::Ahrs ahrs{gyro, acc, mag, dt};
};

