#include "ahrs/numeric.h"

#include <gmock/gmock-matchers.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace ::testing;
using namespace ahrs;

MATCHER_P(Arrays2dDoubleEq, expected, "") {
    for (int i = 0; i < arg.size(); i++) {
        for (int j = 0; j < arg[i].size(); j++) {
            EXPECT_THAT(arg[i][j], DoubleEq(expected[i][j]));
        }
    }
    return true;
}

MATCHER_P2(Arrays2dDoubleNear, expected, max_abs_err, "") {
    for (int i = 0; i < arg.size(); i++) {
        for (int j = 0; j < arg[i].size(); j++) {
            EXPECT_THAT(arg[i][j], DoubleNear(expected[i][j], max_abs_err));
        }
    }
    return true;
}

TEST(EkfNumericTest, Creates3By3EyeMatrix) {
    ahrs::array_2d<int, 3, 3> expected = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    ahrs::array_2d<int, 3, 3> result = ahrs::eye<int, 3>();
    EXPECT_EQ(result, expected);
}

TEST(EkfNumericTest, Creates3By3ZerosMatrix) {
    ahrs::array_2d<int, 3, 3> expected = {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}};
    ahrs::array_2d<int, 3, 3> result = ahrs::zeros<int, 3, 3>();
    EXPECT_EQ(result, expected);
}

TEST(EkfNumericTest, Adds3By3And3By3Correctly) {
    ahrs::array_2d<double, 3, 3> lhs = {
        {{9.11, 18.09, 10.28}, {6.57, 0.10, 11.99}, {15.77, 19.14, 12.43}}};
    ahrs::array_2d<double, 3, 3> rhs = {
        {{6.94, 6.89, 24.97}, {21.23, 5.20, 5.12}, {2.49, 24.10, 10.85}}};

    ahrs::array_2d<double, 3, 3> expected = {
        {{16.05, 24.98, 35.25}, {27.80, 5.30, 17.11}, {18.26, 43.24, 23.28}}};

    EXPECT_THAT(lhs + rhs, Arrays2dDoubleEq(expected));
}

TEST(EkfNumericTest, Subs3By3And3By3Correctly) {
    ahrs::array_2d<double, 3, 3> lhs = {
        {{9.11, 18.09, 10.28}, {6.57, 0.10, 11.99}, {15.77, 19.14, 12.43}}};
    ahrs::array_2d<double, 3, 3> rhs = {
        {{6.94, 6.89, 24.97}, {21.23, 5.20, 5.12}, {2.49, 24.10, 10.85}}};

    ahrs::array_2d<double, 3, 3> expected = {
        {{2.17, 11.20, -14.69}, {-14.66, -5.10, 6.87}, {13.28, -4.96, 1.58}}};

    EXPECT_THAT(lhs - rhs, Arrays2dDoubleEq(expected));
}

TEST(EkfNumericTest, Muls3by1And1By3Correclty) {
    ahrs::array_2d<double, 3, 1> lhs = {{{6.94}, {6.89}, {24.97}}};
    ahrs::array_2d<double, 1, 3> rhs = {{{9.11, 18.09, 10.28}}};

    ahrs::array_2d<double, 3, 3> expected = {{{63.2234, 125.5446, 71.3432},
                                              {62.7679, 124.6401, 70.8292},
                                              {227.4767, 451.7073, 256.6916}}};

    EXPECT_THAT(lhs * rhs, Arrays2dDoubleEq(expected));
}

TEST(EkfNumericTest, Muls1by3And3By1Correclty) {
    ahrs::array_2d<double, 1, 3> lhs = {{{9.11, 18.09, 10.28}}};
    ahrs::array_2d<double, 3, 1> rhs = {{{6.94}, {6.89}, {24.97}}};

    ahrs::array_2d<double, 1, 1> expected = {{{444.5551}}};

    EXPECT_THAT(lhs * rhs, Arrays2dDoubleEq(expected));
}
TEST(EkfNumericTest, Muls3By3And3By3Correctly) {
    ahrs::array_2d<double, 3, 3> lhs = {
        {{9.11, 18.09, 10.28}, {6.57, 0.10, 11.99}, {15.77, 19.14, 22.43}}};
    ahrs::array_2d<double, 3, 3> rhs = {
        {{6.94, 6.89, 24.97}, {21.23, 5.20, 5.12}, {2.49, 24.10, 20.85}}};

    ahrs::array_2d<double, 3, 3> expected = {{{472.8713, 404.5839, 534.4355},
                                              {77.5739, 334.7463, 414.5564},
                                              {571.6367, 748.7463, 959.4392}}};

    EXPECT_THAT(lhs * rhs, Arrays2dDoubleEq(expected));
}

TEST(EkfNumericTest, Muls3By3And3By1Correctly) {
    ahrs::array_2d<double, 3, 3> lhs = {
        {{9.11, 18.09, 10.28}, {6.57, 0.10, 11.99}, {15.77, 19.14, 22.43}}};
    ahrs::array_2d<double, 3, 3> rhs = {{{6.94}, {6.89}, {24.97}}};

    ahrs::array_2d<double, 3, 3> expected = {
        {{444.5551}, {345.6751}, {801.3955}}};

    EXPECT_THAT(lhs * rhs, Arrays2dDoubleEq(expected));
}

TEST(EkfNumericTest, Muls3By1And3By3Correctly) {
    ahrs::array_2d<double, 3, 3> lhs = {{6.94, 6.89, 24.97}};
    ahrs::array_2d<double, 3, 3> rhs = {
        {{9.11, 18.09, 10.28}, {6.57, 0.10, 11.99}, {15.77, 19.14, 22.43}}};

    ahrs::array_2d<double, 3, 3> expected = {{502.2676, 604.1594, 714.0314}};

    EXPECT_THAT(lhs * rhs, Arrays2dDoubleEq(expected));
}

TEST(EkfNumericTest, Muls1By3And3By1Correctly) {
    ahrs::array_2d<double, 3, 3> lhs = {{6.94, 6.89, 24.97}};
    ahrs::array_2d<double, 3, 3> rhs = {{{9.11}, {6.57}, {15.77}}};

    ahrs::array_2d<double, 3, 3> expected = {{502.2676}};

    EXPECT_THAT(lhs * rhs, Arrays2dDoubleEq(expected));
}

TEST(EkfNumericTest, Muls3By1And1By3Correctly) {
    ahrs::array_2d<double, 3, 3> lhs = {{{9.11}, {6.57}, {15.77}}};
    ahrs::array_2d<double, 3, 3> rhs = {{6.94, 6.89, 24.97}};

    ahrs::array_2d<double, 3, 3> expected = {{{63.2234, 62.7679, 227.4767},
                                              {45.5958, 45.2673, 164.0529},
                                              {109.4438, 108.6553, 393.7769}}};

    EXPECT_THAT(lhs * rhs, Arrays2dDoubleEq(expected));
}

TEST(EkfNumericTest, Transposes3By3Correctly) {
    ahrs::array_2d<double, 3, 3> arr = {
        {{9.11, 18.09, 10.28}, {6.57, 0.10, 11.99}, {15.77, 19.14, 22.43}}};

    ahrs::array_2d<double, 3, 3> expected = {
        {{9.11, 6.57, 15.77}, {18.09, 0.10, 19.14}, {10.28, 11.99, 22.43}}};

    EXPECT_THAT(ahrs::transpose(arr), Arrays2dDoubleEq(expected));
}

TEST(EkfNumericTest, AddsIdentityRightTo3By3Matrix) {
    ahrs::array_2d<double, 3, 3> arr = {
        {{5.0, 7.0, 9.0}, {4.0, 3.0, 8.0}, {7.0, 5.0, 6.0}}};

    ahrs::array_2d<double, 3, 6> expected = {{{5.0, 7.0, 9.0, 1, 0, 0},
                                              {4.0, 3.0, 8.0, 0, 1, 0},
                                              {7.0, 5.0, 6.0, 0, 0, 1}}};

    EXPECT_THAT(add_identity(arr), expected);
}

TEST(EkfNumericTest, PerformsGaussSwappingOn3By3) {
    ahrs::array_2d<double, 3, 6> arr = {{{5.0, 7.0, 9.0, 1, 0, 0},
                                         {4.0, 3.0, 8.0, 0, 1, 0},
                                         {7.0, 5.0, 6.0, 0, 0, 1}}};

    ahrs::array_2d<double, 3, 6> expected = {{{7.0, 5.0, 6.0, 0, 0, 1.0},
                                              {5.0, 7.0, 9.0, 1.0, 0, 0},
                                              {4.0, 3.0, 8.0, 0, 1.0, 0}}};

    EXPECT_THAT(ahrs::gauss_swap(arr), Arrays2dDoubleEq(expected));
}

TEST(EkfNumericTest, PerformsGaussReductionToIdentityOn3By3) {
    ahrs::array_2d<double, 3, 6> arr = {{{7.0, 5.0, 6.0, 0, 0, 1.0},
                                         {5.0, 7.0, 9.0, 1.0, 0, 0},
                                         {4.0, 3.0, 8.0, 0, 1.0, 0}}};

    ahrs::array_2d<double, 3, 6> expected = {
        {{1.0, 0, 0, -0.210, 0.029, 0.276},
         {0, 1.0, 0, 0.305, -0.314, -0.038},
         {0, 0, 1.0, -0.010, 0.229, -0.124}}};

    EXPECT_THAT(ahrs::gauss_reduce(arr), Arrays2dDoubleNear(expected, 0.01));
}

TEST(EkfNumericTest, ExtractsInvertedFrom3By3Extended) {
    ahrs::array_2d<double, 3, 6> arr = {{{1.0, 0, 0, -0.210, 0.029, 0.276},
                                         {0, 1.0, 0, 0.305, -0.314, -0.038},
                                         {0, 0, 1.0, -0.010, 0.229, -0.124}}};

    ahrs::array_2d<double, 3, 3> expected = {{{-0.210, 0.029, 0.276},
                                              {0.305, -0.314, -0.038},
                                              {-0.010, 0.229, -0.124}}};

    EXPECT_THAT(ahrs::extract_inv(arr), Arrays2dDoubleEq(expected));
}

TEST(EkfNumericTest, Inverts3by3) {
    ahrs::array_2d<double, 3, 3> arr = {
        {{5.0, 7.0, 9.0}, {4.0, 3.0, 8.0}, {7.0, 5.0, 6.0}}};

    ahrs::array_2d<double, 3, 3> expected = {{{-0.210, 0.029, 0.276},
                                              {0.305, -0.314, -0.038},
                                              {-0.010, 0.229, -0.124}}};

    EXPECT_THAT(ahrs::inv(arr), Arrays2dDoubleNear(expected, 0.01));
}
