#include <gmock/gmock-spec-builders.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "ahrs/numeric.h"
#include "mocks.h"

using namespace ::testing;
using ::testing::Return;
using namespace ahrs;

class ImuCalibratedSensorTest : public Test {
   public:
    StrictMock<SensorMock> sensor_mock;
    ImuCalibratedSensor imu_cal{sensor_mock};
};

class AccelCalibratedSensorTest : public Test {
   public:
    SensorMock sensor_mock;
    AccelCalibratedSensor accel_cal{sensor_mock};
};

class CompassCalibratedSensorTest : public Test {
   public:
    SensorMock sensor_mock;
    CompassCalibratedSensor comp_cal{sensor_mock};
};

TEST_F(ImuCalibratedSensorTest, FixesBiasOnRead) {
    imu_cal.offset_bias = {1, 2, 3};
    sensor_readout readout = {10, 20, 30};
    sensor_readout expected = {10 - 1, 20 - 2, 30 - 3};

    EXPECT_CALL(sensor_mock, read()).WillOnce(Return(readout));

    auto res = imu_cal.read();
    EXPECT_EQ(res, expected);
}

TEST_F(ImuCalibratedSensorTest, CalibratesBias) {
    /* Bias should be avg of readouts */
    sensor_readout expected = {-5, 5, 5};

    InSequence s;
    EXPECT_CALL(sensor_mock, read())
        .Times(25)
        .WillRepeatedly(Return(sensor_readout{-2, 2, 3}));
    EXPECT_CALL(sensor_mock, read())
        .Times(25)
        .WillRepeatedly(Return(sensor_readout{-4, 4, 7}));
    EXPECT_CALL(sensor_mock, read())
        .Times(25)
        .WillRepeatedly(Return(sensor_readout{-6, 6, 10}));
    EXPECT_CALL(sensor_mock, read())
        .Times(25)
        .WillRepeatedly(Return(sensor_readout{-8, 8, 0}));

    imu_cal.calibrate_bias(100);

    EXPECT_EQ(imu_cal.offset_bias, expected);
}

TEST_F(AccelCalibratedSensorTest, ZeroesZAxisBiasAfterCalibration) {
    /* Bias should be avg of readouts */
    sensor_readout expected = {-5, 5, 0};

    InSequence s;
    EXPECT_CALL(sensor_mock, read())
        .Times(25)
        .WillRepeatedly(Return(sensor_readout{-2, 2, 3}));
    EXPECT_CALL(sensor_mock, read())
        .Times(25)
        .WillRepeatedly(Return(sensor_readout{-4, 4, 7}));
    EXPECT_CALL(sensor_mock, read())
        .Times(25)
        .WillRepeatedly(Return(sensor_readout{-6, 6, 10}));
    EXPECT_CALL(sensor_mock, read())
        .Times(25)
        .WillRepeatedly(Return(sensor_readout{-8, 8, 0}));

    accel_cal.calibrate_bias(100);

    EXPECT_EQ(accel_cal.offset_bias, expected);
}

TEST_F(CompassCalibratedSensorTest, FixesBiasOnRead) {
    comp_cal.hard_iron_bias = {11, 12, 13};
    comp_cal.soft_iron_bias = {1, 2, 3};
    sensor_readout readout = {10, 20, 30};
    sensor_readout expected = {(10 - 11) * 1, (20 - 12) * 2, (30 - 13) * 3};

    EXPECT_CALL(sensor_mock, read()).WillOnce(Return(readout));

    auto res = comp_cal.read();
    EXPECT_EQ(res, expected);
}

TEST_F(CompassCalibratedSensorTest,
       CalibrationCallsProperNumberOfReadoutsOnHardAndSoftIron) {
    EXPECT_CALL(sensor_mock, read()).Times(2 * 100);
    comp_cal.calibrate_bias(100);
}

TEST_F(CompassCalibratedSensorTest, CalibratesHardIron) {
    sensor_readout expected = {5, 10, 15};

    InSequence s;
    /* x max, y, z */
    EXPECT_CALL(sensor_mock, read())
        .Times(25)
        .WillRepeatedly(Return(sensor_readout{11, 2, 3}));
    /* x min, y max, z */
    EXPECT_CALL(sensor_mock, read())
        .Times(25)
        .WillRepeatedly(Return(sensor_readout{-1, 22, 7}));
    /* x, y min, z max */
    EXPECT_CALL(sensor_mock, read())
        .Times(25)
        .WillRepeatedly(Return(sensor_readout{2, -2, 33}));
    /* x, y, z min */
    EXPECT_CALL(sensor_mock, read())
        .Times(25)
        .WillRepeatedly(Return(sensor_readout{5, 8, -3}));

    comp_cal.calibrate_hard_iron(100);

    EXPECT_EQ(comp_cal.hard_iron_bias, expected);
}

TEST_F(CompassCalibratedSensorTest, CalibratesSoftIron) {
    sensor_readout expected = {(8. + 4. + 2.) / 3 / 8,
                               (8. + 4. + 2.) / 3 / 4,
                               (8. + 4. + 2.) / 3 / 2};

    InSequence s;
    /* x max, y, z */
    EXPECT_CALL(sensor_mock, read())
        .Times(25)
        .WillRepeatedly(Return(sensor_readout{8, -1, 1}));
    /* x min, y max, z */
    EXPECT_CALL(sensor_mock, read())
        .Times(25)
        .WillRepeatedly(Return(sensor_readout{-8, 4, -1}));
    /* x, y min, z max */
    EXPECT_CALL(sensor_mock, read())
        .Times(25)
        .WillRepeatedly(Return(sensor_readout{1, -4, 2}));
    /* x, y, z min */
    EXPECT_CALL(sensor_mock, read())
        .Times(25)
        .WillRepeatedly(Return(sensor_readout{-1, 1, -2}));

    comp_cal.calibrate_soft_iron(100);

    EXPECT_EQ(comp_cal.soft_iron_bias, expected);
}
