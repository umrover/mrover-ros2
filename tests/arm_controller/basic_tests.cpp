#include "harness.hpp"

namespace mrover {
    TEST_F(ArmControllerHarness, ArmPos) {
        ArmPos pos1;

        pos1 = pos1 + R3d{-1, 2, 3};
        EXPECT_EQ(pos1.x, -1);
        EXPECT_EQ(pos1.y, 2);
        EXPECT_EQ(pos1.z, 3);

        msg::IK ik;
        ik.pos.x = 1;
        ik.pos.y = 2;
        ik.pos.z = 3;
        ik.pitch = 4;
        ik.roll = 5;
        ArmPos pos2;
        pos2 = ik;
        EXPECT_EQ(pos2.x, 1);
        EXPECT_EQ(pos2.y, 2);
        EXPECT_EQ(pos2.z, 3);
        EXPECT_EQ(pos2.pitch, 4);
        EXPECT_EQ(pos2.roll, 5);

        // test conversion to SE3
        ArmPos pos3;
        SE3d se3 = pos3.toSE3();
        EXPECT_EQ(se3, SE3d::Identity());

        se3 = pos2.toSE3();
        EXPECT_EQ(se3.x(), 1);
        EXPECT_EQ(se3.y(), 2);
        EXPECT_EQ(se3.z(), 3);
    }

    TEST_F(ArmControllerHarness, JointWrapper) {
        JointWrapper joint{
            .limits = { .minPos = 0, .maxPos = 1, .minVel = -1, .maxVel = 1},
            .pos = 0
        };

        EXPECT_TRUE(joint.limits.posInBounds(0.5));
        EXPECT_FALSE(joint.limits.posInBounds(-0.5));
        EXPECT_FALSE(joint.limits.posInBounds(1.5));

        EXPECT_TRUE(joint.limits.velInBounds(0.5));
        EXPECT_TRUE(joint.limits.velInBounds(-0.5));
        EXPECT_FALSE(joint.limits.velInBounds(2));
        EXPECT_FALSE(joint.limits.velInBounds(-1.5));
    }

    void ArmControllerHarness::testPosCallback() {
        auto msg = std::make_shared<msg::IK>();
        msg->pos.x = 1;
        msg->pos.y = 2;
        msg->pos.z = 2;
        msg->pitch = 4;
        msg->roll = 5;
        ArmPos pos;
        pos = *msg;

        rclcpp::Time old_time = node->get_clock()->now();
        node->posCallback(msg);
        rclcpp::Time new_time = node->get_clock()->now();
        checkPos(pos, node->mPosTarget);
        EXPECT_GT(node->mLastUpdate, old_time);
        EXPECT_LT(node->mLastUpdate, new_time);

        // in velocity mode we shouldn't update timestamp
        node->mArmMode = ArmController::ArmMode::VELOCITY_CONTROL;
        node->posCallback(msg);
        EXPECT_LT(node->mLastUpdate, new_time);
    }

    void ArmControllerHarness::testVelCallback() {
        auto msg = std::make_shared<geometry_msgs::msg::Twist>();

        // default is in position control mode
        rclcpp::Time old_time = node->get_clock()->now();
        node->velCallback(msg);
        EXPECT_EQ(node->mVelTarget, *msg);
        EXPECT_LT(node->mLastUpdate, old_time);

        msg->linear.x = 0.1;
        msg->linear.y = 0.2;
        msg->linear.z = 1.0;
        msg->angular.x = 1;
        msg->angular.y = 2;
        msg->angular.z = 3;
        node->mArmMode = ArmController::ArmMode::VELOCITY_CONTROL;
        node->velCallback(msg);
        EXPECT_EQ(node->mVelTarget.angular, msg->angular);
        EXPECT_FLOAT_EQ(msg->linear.x * ArmController::MAX_SPEED, node->mVelTarget.linear.x);
        EXPECT_FLOAT_EQ(msg->linear.y * ArmController::MAX_SPEED, node->mVelTarget.linear.y);
        EXPECT_FLOAT_EQ(msg->linear.z * ArmController::MAX_SPEED, node->mVelTarget.linear.z);
        EXPECT_GT(node->mLastUpdate, old_time);
    }

    void ArmControllerHarness::testHandleGoal() {
        rclcpp_action::GoalUUID uuid;
        auto goal = std::make_shared<action::TypingPosition_Goal>();

        // arm controller starts in position IK mode so we expect rejection
        auto resp = node->handleTypingGoal(uuid, goal);
        EXPECT_EQ(resp, rclcpp_action::GoalResponse::REJECT);

        node->mArmMode = ArmController::ArmMode::TYPING;
        resp = node->handleTypingGoal(uuid, goal);
        EXPECT_EQ(resp, rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);

        // repeat request while prior is in flight should fail
        resp = node->handleTypingGoal(uuid, goal);
        EXPECT_EQ(resp, rclcpp_action::GoalResponse::REJECT);
    }

    void ArmControllerHarness::testModeCallback() {
        auto req = std::make_shared<srv::IkMode::Request>();
        auto resp = std::make_shared<srv::IkMode::Response>();
        req->mode = srv::IkMode::Request::POSITION_CONTROL;
        node->modeCallback(req, resp);
        EXPECT_EQ(node->mArmMode, ArmController::ArmMode::POSITION_CONTROL);
        EXPECT_TRUE(resp->success);
        resp->success = false;

        req->mode = srv::IkMode::Request::VELOCITY_CONTROL;
        node->modeCallback(req, resp);
        EXPECT_EQ(node->mArmMode, ArmController::ArmMode::VELOCITY_CONTROL);
        EXPECT_TRUE(resp->success);
        resp->success = false;

        req->mode = srv::IkMode::Request::TYPING;
        node->modeCallback(req, resp);
        EXPECT_EQ(node->mArmMode, ArmController::ArmMode::TYPING);
        EXPECT_TRUE(resp->success);
        resp->success = false;
    }

    TEST_F(ArmControllerHarness, posCallback) {
        testPosCallback();
    }

    TEST_F(ArmControllerHarness, velCallback) {
        testVelCallback();
    }

    TEST_F(ArmControllerHarness, handleGoal) {
        testHandleGoal();
    }

    TEST_F(ArmControllerHarness, modeCallback) {
        testModeCallback();
    }
}
