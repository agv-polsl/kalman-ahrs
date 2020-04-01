#include "ekf_ahrs/ekf_ahrs.hpp"

#include <gtest/gtest.h>

using namespace ::testing;

TEST(KalmanFilterTest, CalculatesPitchFromAccel) {
    double acc_x = 0.3256;
    double acc_z = -0.6646;

    double expected = 0.5363;

    EXPECT_NEAR(ekfn::calc_pitch(acc_x, acc_z), expected, 0.0001);
}

TEST(KalmanFilterTest, CalculatesRollFromAccel) {
    double acc_y = 9.8569;
    double acc_z = -0.6646;

    double expected = 0.1007;

    EXPECT_NEAR(ekfn::calc_pitch(acc_y, acc_z), expected, 0.0001);
}
