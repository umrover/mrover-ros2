#include <gtest/gtest.h>
#include "../arm_controller.hpp"

namespace mrover {
    class TestArmController : public ::testing::Test {
    protected:

        static void SetUpTestSuite()
        {
            rclcpp::init(0, nullptr);
        }

        static void TearDownTestSuite()
        {
            rclcpp::shutdown();
        }

        static void testIkPosCalc() {
            ArmController arm_controller;

            // something unreachable cause links are too short
            ASSERT_EQ(arm_controller.ikPosCalc(ArmController::ArmPos{10000}), std::nullopt);

            // something unreachable cause insufficient angular ROM
            ASSERT_EQ(arm_controller.ikPosCalc(ArmController::ArmPos{0.5}), std::nullopt);

            // something reachable
            auto res = arm_controller.ikPosCalc(ArmController::ArmPos{0.7});
            ASSERT_TRUE(res.has_value());
        }

        static void testArmPos() {
            ArmController::ArmPos pos1;

            pos1 = pos1 + R3d{-1, 2, 3};
            ASSERT_EQ(pos1.x, -1);
            ASSERT_EQ(pos1.y, 2);
            ASSERT_EQ(pos1.z, 3);

            msg::IK ik;
            ik.pos.x = 1;
            ik.pos.y = 2;
            ik.pos.z = 3;
            ik.pitch = 4;
            ik.roll = 5;
            ArmController::ArmPos pos2;
            pos2 = ik;
            ASSERT_EQ(pos2.x, 1);
            ASSERT_EQ(pos2.y, 2);
            ASSERT_EQ(pos2.z, 3);
            ASSERT_EQ(pos2.pitch, 4);
            ASSERT_EQ(pos2.roll, 5);
        }

        static void testJointWrapper() {
            ArmController::JointWrapper joint{
                .limits = { .minPos = 0, .maxPos = 1, .minVel = -1, .maxVel = 1},
                .pos = 0
            };

            ASSERT_TRUE(joint.limits.posInBounds(0.5));
            ASSERT_FALSE(joint.limits.posInBounds(-0.5));
            ASSERT_FALSE(joint.limits.posInBounds(1.5));

            ASSERT_TRUE(joint.limits.velInBounds(0.5));
            ASSERT_TRUE(joint.limits.velInBounds(-0.5));
            ASSERT_FALSE(joint.limits.velInBounds(2));
            ASSERT_FALSE(joint.limits.velInBounds(-1.5));
        }
    };

    TEST_F(TestArmController, ikPosCalc) {
        testIkPosCalc();
    }

    TEST_F(TestArmController, ArmPos) {
        testArmPos();
    }

    TEST_F(TestArmController, JointWrapper) {
        testJointWrapper();
    }
}
