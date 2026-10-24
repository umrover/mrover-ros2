#pragma once
#include <gtest/gtest.h>
#include "navigation/arm_controller/arm_controller.hpp"

namespace mrover {
    class ArmControllerHarness : public ::testing::Test {
        ArmController arm_controller;

    protected:
        // these using statements are needed so these private types are
        // accessible inside TEST_F functions
        using ArmPos = ArmController::ArmPos;
        using JointWrapper = ArmController::JointWrapper;

        static void SetUpTestSuite()
        {
            rclcpp::init(0, nullptr);
        }

        static void TearDownTestSuite()
        {
            rclcpp::shutdown();
        }

        static void checkPos(ArmPos pos1, ArmPos pos2) {
            EXPECT_FLOAT_EQ(pos1.x, pos2.x);
            EXPECT_FLOAT_EQ(pos1.y, pos2.y);
            EXPECT_FLOAT_EQ(pos1.z, pos2.z);
            EXPECT_FLOAT_EQ(pos1.pitch, pos2.pitch);
            EXPECT_FLOAT_EQ(pos1.roll, pos2.roll);
            EXPECT_FLOAT_EQ(pos1.gripper, pos2.gripper);
        }

        void testPosCallback();
        void testVelCallback();

        void testIkPosCalc();
        void testIkFk();
        void testIkVelCalc();
    };
}
