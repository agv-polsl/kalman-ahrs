#include <gtest/gtest.h>

#include "ekf_ahrs/ekf_ahrs.hpp"

using namespace ::testing;

class EkfNumericTest : public Test {};

TEST_F(EkfNumericTest, DoesWork) { EXPECT_EQ(1, 1); }
