#include "ahrs/sensor_readout.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace ::testing;
using namespace ahrs;

TEST(SensorReadoutTest, EqualsOperatorOverloads) {
    sensor_readout lhs = {1, 2, 3};
    sensor_readout rhs = {1, 2, 3};
    EXPECT_TRUE(lhs == rhs);
}

TEST(SensorReadoutTest, NotEqualsOperatorOverloads) {
    sensor_readout lhs = {1, 2, 3};
    sensor_readout rhs = {10, 2, 3};
    EXPECT_TRUE(lhs != rhs);
}

TEST(SensorReadoutTest, PlusOperatorOverloads) {
    sensor_readout lhs = {1, 2, 3};
    sensor_readout rhs = {10, 20, 30};
    sensor_readout expected = {11, 22, 33};
    EXPECT_EQ(lhs + rhs, expected);
}

TEST(SensorReadoutTest, PlusEqualsOperatorOverloads) {
    sensor_readout lhs = {1, 2, 3};
    sensor_readout rhs = {10, 20, 30};
    sensor_readout expected = {11, 22, 33};
    EXPECT_EQ(lhs += rhs, expected);
}

TEST(SensorReadoutTest, MinusOperatorOverloads) {
    sensor_readout lhs = {1, 2, 3};
    sensor_readout rhs = {10, 20, 30};
    sensor_readout expected = {-9, -18, -27};
    EXPECT_EQ(lhs - rhs, expected);
}

TEST(SensorReadoutTest, MinusEqualsOperatorOverloads) {
    sensor_readout lhs = {1, 2, 3};
    sensor_readout rhs = {10, 20, 30};
    sensor_readout expected = {-9, -18, -27};
    EXPECT_EQ(lhs -= rhs, expected);
}

TEST(SensorReadoutTest, MulOperatorOverloads) {
    sensor_readout lhs = {1, 2, 3};
    sensor_readout rhs = {10, 20, 30};
    sensor_readout expected = {10, 40, 90};
    EXPECT_EQ(lhs * rhs, expected);
}

TEST(SensorReadoutTest, MulEqualsOperatorOverloads) {
    sensor_readout lhs = {1, 2, 3};
    sensor_readout rhs = {10, 20, 30};
    sensor_readout expected = {10, 40, 90};
    EXPECT_EQ(lhs *= rhs, expected);
}

TEST(SensorReadoutTest, DivOperatorOverloads) {
    sensor_readout lhs = {1, 2, 3};
    sensor_readout rhs = {10, 20, 30};
    sensor_readout expected = {0.1, 0.1, 0.1};
    EXPECT_EQ(lhs / rhs, expected);
}

TEST(SensorReadoutTest, DivEqualsOperatorOverloads) {
    sensor_readout lhs = {1, 2, 3};
    sensor_readout rhs = {10, 20, 30};
    sensor_readout expected = {0.1, 0.1, 0.1};
    EXPECT_EQ(lhs /= rhs, expected);
}
