#include "ekf_ahrs/numeric.hpp"

#include <gtest/gtest.h>

using namespace ::testing;
using namespace ekfn;

class EkfNumericTest : public Test {};

TEST_F(EkfNumericTest, Muls3By3And3By3Correctly) {
    ekfn::array_2d<double, 3, 3> lhs = {
        {{9.11, 18.09, 10.28}, {6.57, 0.10, 11.99}, {15.77, 19.14, 22.43}}};
    ekfn::array_2d<double, 3, 3> rhs = {
        {{6.94, 6.89, 24.97}, {21.23, 5.20, 5.12}, {2.49, 24.10, 20.85}}};

    ekfn::array_2d<double, 3, 3> res = {{{472.8713, 404.5839, 534.4355},
                                         {77.5739, 334.7463, 414.5564},
                                         {571.6367, 748.7463, 959.4392}}};

    ASSERT_EQ(lhs * rhs, res);
}

TEST(EkfNumericTest, Muls3By3And3By1Correctly) {
    ekfn::array_2d<double, 3, 3> lhs = {
        {{9.11, 18.09, 10.28}, {6.57, 0.10, 11.99}, {15.77, 19.14, 22.43}}};
    ekfn::array_2d<double, 3, 3> rhs = {{{6.94}, {6.89}, {24.97}}};

    ekfn::array_2d<double, 3, 3> res = {{{444.5551}, {345.6751}, {801.3955}}};

    ASSERT_EQ(lhs * rhs, res);
}

TEST(EkfNumericTest, Muls3By1And3By3Correctly) {
    ekfn::array_2d<double, 3, 3> lhs = {{6.94, 6.89, 24.97}};
    ekfn::array_2d<double, 3, 3> rhs = {
        {{9.11, 18.09, 10.28}, {6.57, 0.10, 11.99}, {15.77, 19.14, 22.43}}};

    ekfn::array_2d<double, 3, 3> res = {{502.2676, 604.1594, 714.0314}};

    ASSERT_EQ(lhs * rhs, res);
}

TEST(EkfNumericTest, Muls1By3And3By1Correctly) {
    ekfn::array_2d<double, 3, 3> lhs = {{6.94, 6.89, 24.97}};
    ekfn::array_2d<double, 3, 3> rhs = {{{9.11}, {6.57}, {15.77}}};

    ekfn::array_2d<double, 3, 3> res = {{502.2676}};

    ASSERT_EQ(lhs * rhs, res);
}

TEST(EkfNumericTest, Muls3By1And1By3Correctly) {
    ekfn::array_2d<double, 3, 3> lhs = {{{9.11}, {6.57}, {15.77}}};
    ekfn::array_2d<double, 3, 3> rhs = {{6.94, 6.89, 24.97}};

    ekfn::array_2d<double, 3, 3> res = {{{63.2234, 62.7679, 227.4767},
                                         {45.5958, 45.2673, 164.0529},
                                         {109.4438, 108.6553, 393.7769}}};

    ASSERT_EQ(lhs * rhs, res);
}

TEST(EkfNumericTest, Transposes3By3Correctly) {
    ekfn::array_2d<double, 3, 3> arr = {
        {{9.11, 18.09, 10.28}, {6.57, 0.10, 11.99}, {15.77, 19.14, 22.43}}};

    ekfn::array_2d<double, 3, 3> res = {
        {{9.11, 6.57, 15.77}, {18.09, 0.10, 19.14}, {10.28, 11.99, 22.43}}};

    ASSERT_EQ(ekfn::transpose(arr), res);
}

TEST(EkfNumericTest, AddsIdentityRightTo3By3Matrix) {
    ekfn::array_2d<double, 3, 3> arr = {
        {{5.0, 7.0, 9.0}, {4.0, 3.0, 8.0}, {7.0, 5.0, 6.0}}};

    ekfn::array_2d<double, 3, 6> res = {{{5.0, 7.0, 9.0, 1, 0, 0},
                                         {4.0, 3.0, 8.0, 0, 1, 0},
                                         {7.0, 5.0, 6.0, 0, 0, 1}}};

    ASSERT_EQ(ekfn::add_identity(arr), res);
}

TEST(EkfNumericTest, PerformsGaussSwappingOn3By3) {
    ekfn::array_2d<double, 3, 6> arr = {{{5.0, 7.0, 9.0, 1, 0, 0},
                                         {4.0, 3.0, 8.0, 0, 1, 0},
                                         {7.0, 5.0, 6.0, 0, 0, 1}}};

    ekfn::array_2d<double, 3, 6> res = {{{7.0, 5.0, 6.0, 0, 0, 1.0},
                                         {5.0, 7.0, 9.0, 1.0, 0, 0},
                                         {4.0, 3.0, 8.0, 0, 1.0, 0}}};

    ASSERT_EQ(ekfn::gauss_swap(arr), res);
}

TEST(EkfNumericTest, PerformsGaussReductionToIdentityOn3By3) {
    ekfn::array_2d<double, 3, 6> arr = {{{7.0, 5.0, 6.0, 0, 0, 1.0},
                                         {5.0, 7.0, 9.0, 1.0, 0, 0},
                                         {4.0, 3.0, 8.0, 0, 1.0, 0}}};

    ekfn::array_2d<double, 3, 6> res = {{{1.0, 0, 0, -0.210, 0.029, 0.276},
                                         {0, 1.0, 0, 0.305, -0.314, -0.038},
                                         {0, 0, 1.0, -0.010, 0.229, -0.124}}};

    ASSERT_EQ(ekfn::gauss_reduce(arr), res);
}

TEST(EkfNumericTest, ExtractsInvertedFrom3By3Extended) {
    ekfn::array_2d<double, 3, 6> arr = {{{1.0, 0, 0, -0.210, 0.029, 0.276},
                                         {0, 1.0, 0, 0.305, -0.314, -0.038},
                                         {0, 0, 1.0, -0.010, 0.229, -0.124}}};

    ekfn::array_2d<double, 3, 3> res = {{{-0.210, 0.029, 0.276},
                                         {0.305, -0.314, -0.038},
                                         {-0.010, 0.229, -0.124}}};

    ASSERT_EQ(ekfn::extract_inv(arr), res);
}

TEST(EkfNumericTest, Inverts3by3) {
    ekfn::array_2d<double, 3, 3> arr = {
        {{5.0, 7.0, 9.0}, {4.0, 3.0, 8.0}, {7.0, 5.0, 6.0}}};

    ekfn::array_2d<double, 3, 3> res = {{{-0.210, 0.029, 0.276},
                                         {0.305, -0.314, -0.038},
                                         {-0.010, 0.229, -0.124}}};
    ASSERT_EQ(ekfn::inv(arr), res);
}
