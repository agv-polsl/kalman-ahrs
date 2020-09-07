#include "ahrs/ahrs.h"

#include <gmock/gmock-nice-strict.h>
#include <gtest/gtest.h>
#include <chrono>

#include "mocks.h"

using namespace std::chrono_literals;
using namespace ::testing;
using namespace ahrs;

class AhrsTest : public Test {
   public:
    NiceMock<SensorMock> gyro;
    NiceMock<SensorMock> acc;
    NiceMock<SensorMock> mag;
    std::chrono::milliseconds dt = 10ms;

    ahrs::Ahrs ahrs{gyro, acc, mag, dt};
};

/*
 * This is prepared for testing AHRS system. Since predictions of the system
 * are based on statistics and complex calculations they are demanding to test.
 * It would be best to get a professional AHRS system and compare its output
 * with the code (keeping in mind a sensible margin of error).
 */
