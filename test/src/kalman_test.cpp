#include "ahrs/kalman.h"

#include <gtest/gtest.h>

#include <memory>

using namespace ::testing;
using namespace ahrs;

class KalmanFilterTest : public Test {
   protected:
    std::unique_ptr<ahrs::Kalman> kalman = nullptr;

    void SetUp() override {
        /* The system matrices for this test come from AHRS systems */
        double dt = 0.1;
        ahrs::array_2d<double, 4, 4> A = {
            {{1, -dt, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, -dt}, {0, 0, 0, 1}}};
        ahrs::array_2d<double, 4, 2> B = {{{dt, 0}, {0, 0}, {0, dt}, {0, 0}}};
        ahrs::array_2d<double, 2, 4> H = {{{1, 0, 0, 0}, {0, 0, 1, 0}}};

        kalman = std::make_unique<ahrs::Kalman>(A, B, H);
    }
};

/*
 * This is prepared for testing AHRS system. Since predictions of the system
 * are based on statistics and complex calculations they are demanding to test.
 * It would be best to get a professional AHRS system and compare its output
 * with the code (keeping in mind a sensible margin of error).
 */
