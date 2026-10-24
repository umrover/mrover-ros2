#include "harness.hpp"

namespace mrover {
    void ArmControllerHarness::testIkPosCalc() {
        // something unreachable because links are too short
        EXPECT_EQ(arm_controller.ikPosCalc(ArmPos{10000}), std::nullopt);

        // something unreachable because insufficient angular ROM
        EXPECT_EQ(arm_controller.ikPosCalc(ArmPos{0.5}), std::nullopt);

        // something reachable
        auto res = arm_controller.ikPosCalc(ArmPos{0.7});
        EXPECT_TRUE(res.has_value());

        // test naming mismatch
        arm_controller.joints = {};
        res = arm_controller.ikPosCalc(ArmPos{0.7});
        EXPECT_FALSE(res.has_value());
    }

    void ArmControllerHarness::testIkVelCalc() {
        // force failure
        geometry_msgs::msg::Twist vel;
        vel.angular.y = std::numeric_limits<double>::infinity();
        EXPECT_EQ(arm_controller.ikVelCalc(vel), std::nullopt);

        // test moving out of bounds (arm should be at y=0)
        vel.angular.y = 0;
        vel.linear.y = -1;
        EXPECT_EQ(arm_controller.ikVelCalc(vel), std::nullopt);

        // 0 velocity should yield all zeros
        auto res = arm_controller.ikVelCalc(geometry_msgs::msg::Twist{});
        ASSERT_TRUE(res.has_value());
        for (auto joint_vel : res->velocities) {
            EXPECT_FLOAT_EQ(joint_vel, 0);
        }

        // test naming mismatch
        arm_controller.joints = {};
        res = arm_controller.ikVelCalc(geometry_msgs::msg::Twist{});
        EXPECT_FALSE(res.has_value());
    }

    TEST_F(ArmControllerHarness, ikPosCalc) {
        testIkPosCalc();
    }

    TEST_F(ArmControllerHarness, ikVelCalc) {
        testIkVelCalc();
    }
}
