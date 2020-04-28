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

TEST_F(AhrsTest, CalculatesRollFromAccel) {
    const sensor_readout acc{0.32, -0.1, 1.2};
    const double expected = -0.0803;
    EXPECT_NEAR(ahrs.calc_roll(acc), expected, 0.0001);
}

TEST_F(AhrsTest, CalculatesPitchFromAccel) {
    const sensor_readout acc{0.32, -0.1, 1.2};
    const double expected = -0.2597;
    EXPECT_NEAR(ahrs.calc_pitch(acc), expected, 0.0001);
}

TEST_F(AhrsTest, CalcualtesYawFromMagAndRollPitch) {
    /* This is not implemented due to inconvenience in the
     * reference academic paper.
     */
}
