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

        void testIkPosCalc();

        void testIkVelCalc();
    };
}
