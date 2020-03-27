#include "ekf_ahrs/ekf_ahrs.hpp"

#include <gtest/gtest.h>

using namespace ::testing;

class AhrsTest : public Test {};

TEST_F(AhrsTest, DoesWork) { EXPECT_EQ(1, 1); }
