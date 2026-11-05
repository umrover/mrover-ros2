#include "harness.hpp"

namespace mrover {
    void ArmControllerHarness::testIkPosCalc() {
        // something unreachable because links are too short
        EXPECT_EQ(arm_controller->ikPosCalc(ArmPos{10000}), std::nullopt);

        // something unreachable because insufficient angular ROM
        EXPECT_EQ(arm_controller->ikPosCalc(ArmPos{0.5}), std::nullopt);

        // something reachable
        auto res = arm_controller->ikPosCalc(ArmPos{0.7});
        EXPECT_TRUE(res.has_value());

        // test naming mismatch
        arm_controller->joints = {};
        res = arm_controller->ikPosCalc(ArmPos{0.7});
        EXPECT_FALSE(res.has_value());
    }

    void ArmControllerHarness::testIkFk() {
        // test feeding ik output through fk
        ArmPos pos;
        pos.x = 0.7;
        pos.y = 0.2;
        pos.z = 0.1;
        pos.pitch = 0.1;
        pos.roll = 1;
        pos.gripper = 0;
        auto res = arm_controller->ikPosCalc(pos);
        ASSERT_TRUE(res.has_value());

        auto msg = std::make_shared<msg::ControllerState>();
        msg->names = res->names;
        msg->positions = res->positions;

        arm_controller->fkCallback(msg);
        checkPos(pos, arm_controller->mArmPos);

        // it shouldn't matter which order we pass in the joints
        std::swap(msg->names[0], msg->names[1]);
        std::swap(msg->positions[0], msg->positions[1]);
        arm_controller->fkCallback(msg);
        checkPos(pos, arm_controller->mArmPos);

        // ideally for the following tests we would check how the ArmController
        // responds, but for now we do it just for line coverage
        // test fk with nan position
        msg->positions[0] = std::numeric_limits<float>::quiet_NaN();
        arm_controller->fkCallback(msg);
        msg->positions[0] = 0;

        // test fk with unknown joint
        msg->names.emplace_back("test");
        msg->positions.emplace_back(0);
        arm_controller->fkCallback(msg);
    }

    void ArmControllerHarness::testIkVelCalc() {
        // force failure
        geometry_msgs::msg::Twist vel;
        vel.angular.y = std::numeric_limits<double>::infinity();
        EXPECT_EQ(arm_controller->ikVelCalc(vel), std::nullopt);

        // test moving out of bounds (arm should be at y=0)
        vel.angular.y = 0;
        vel.linear.y = -1;
        EXPECT_EQ(arm_controller->ikVelCalc(vel), std::nullopt);

        // something doable
        vel.linear.y = 0.1;
        vel.linear.x = -0.1;
        vel.linear.z = 0.1;
        EXPECT_TRUE(arm_controller->ikVelCalc(vel).has_value());

        // 0 velocity should yield all zeros
        auto res = arm_controller->ikVelCalc(geometry_msgs::msg::Twist{});
        ASSERT_TRUE(res.has_value());
        for (auto joint_vel : res->velocities) {
            EXPECT_FLOAT_EQ(joint_vel, 0);
        }

        // test naming mismatch
        arm_controller->joints = {};
        res = arm_controller->ikVelCalc(geometry_msgs::msg::Twist{});
        EXPECT_FALSE(res.has_value());
    }

    TEST_F(ArmControllerHarness, ikPosCalc) {
        testIkPosCalc();
    }

    TEST_F(ArmControllerHarness, ikFk) {
        testIkFk();
    }

    TEST_F(ArmControllerHarness, ikVelCalc) {
        testIkVelCalc();
    }
}
