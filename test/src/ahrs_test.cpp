#include "ahrs/ahrs.hpp"

#include <gmock/gmock-nice-strict.h>
#include <gtest/gtest.h>

#include "mocks.hpp"

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

TEST_F(AhrsTest, CalculatesPitchFromAccel) {
    double acc_x = 0.3256;
    double acc_z = -0.6646;

    double expected = 0.5363;

    EXPECT_NEAR(ahrs.calc_pitch(acc_x, acc_z), expected, 0.0001);
}

TEST_F(AhrsTest, CalculatesRollFromAccel) {
    double acc_y = 9.8569;
    double acc_z = -0.6646;

    double expected = 0.1007;

    EXPECT_NEAR(ahrs.calc_pitch(acc_y, acc_z), expected, 0.0001);
}

TEST_F(AhrsTest, CalcualtesYawFromMagAndRollPitch) {
    /* This is not implemented due to inconvenience in the
     * reference academic paper.
     */
}
