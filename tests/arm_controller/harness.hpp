#pragma once
#include <gtest/gtest.h>
#include "navigation/arm_controller/arm_controller.hpp"

namespace mrover {
    class ArmControllerHarness : public ::testing::Test {
        ArmController arm_controller;

    protected:
        static void SetUpTestSuite()
        {
            rclcpp::init(0, nullptr);
        }

        static void TearDownTestSuite()
        {
            rclcpp::shutdown();
        }

        void testIkPosCalc();

        static void testArmPos();

        static void testJointWrapper();
    };
}
