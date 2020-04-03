#include "ekf_ahrs/ekf_ahrs.hpp"

#include <gtest/gtest.h>

#include <iostream>
#include <memory>

using namespace ::testing;
using namespace ekfn;

class AhrsTest : public Test {
   public:
    ekfn::Ahrs ahrs;
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

class KalmanFilterTest : public Test {
   protected:
    std::unique_ptr<ekfn::Kalman> kalman = nullptr;

    void SetUp() override {
        /* The system matrices for this test come from AHRS systems */
        double dt = 0.1;
        ekfn::array_2d<double, 4, 4> A = {
            {{1, -dt, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, -dt}, {0, 0, 0, 1}}};
        ekfn::array_2d<double, 4, 2> B = {{{dt, 0}, {0, 0}, {0, dt}, {0, 0}}};
        ekfn::array_2d<double, 2, 4> H = {{{1, 0, 0, 0}, {0, 0, 1, 0}}};

        kalman = std::make_unique<ekfn::Kalman>(A, B, H, dt);
    }
};

/* TODO: Due to the complex nature of the filter the testing
 * requires much thought and is delegated to the issue #26.
 */
TEST_F(KalmanFilterTest, AnyTest) {}
