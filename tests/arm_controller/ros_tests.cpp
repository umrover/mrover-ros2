#include "harness.hpp"

namespace mrover {
    void ArmControllerRosHarness::changeIkMode(srv::IkMode::Request::_mode_type req) {
        auto request = std::make_shared<srv::IkMode::Request>();
        request->mode = req;
        auto resp = sendRequest<srv::IkMode>(request, "ik_mode");
        ASSERT_TRUE(resp);
        ASSERT_TRUE(resp->success);
    }

    TEST_F(ArmControllerRosHarness, topicTest) {
        const auto topic_map = node->get_topic_names_and_types();
        std::vector<std::pair<std::string, std::string>> expected = {
            {"/arm_pos_cmd", "mrover/msg/Position"},
            {"/arm_vel_cmd", "mrover/msg/Velocity"},
            {"/ik_pos_cmd", "mrover/msg/IK"},
            {"/ik_vel_cmd", "geometry_msgs/msg/Twist"},
            {"/arm_controller_state", "mrover/msg/ControllerState"},
        };

        for (auto& [name, type] : expected) {
            SCOPED_TRACE("Checking topic " + name);
            ASSERT_NE(topic_map.find(name), topic_map.end());
            ASSERT_FALSE(topic_map.at(name).empty());
            ASSERT_EQ(type, topic_map.at(name)[0]);
        }
    }

    TEST_F(ArmControllerRosHarness, serviceTest) {
        const auto service_map = node->get_service_names_and_types();
        std::vector<std::pair<std::string, std::string>> expected = {
            {"/ik_mode", "mrover/srv/IkMode"},
            {"/pusher", "mrover/srv/Pusher"},
        };

        for (auto& [name, type] : expected) {
            SCOPED_TRACE("Checking service " + name);
            ASSERT_NE(service_map.find(name), service_map.end());
            ASSERT_FALSE(service_map.at(name).empty());
            ASSERT_EQ(type, service_map.at(name)[0]);
        }
    }

    TEST_F(ArmControllerRosHarness, subscriptionTest) {
        tf2_ros::Buffer tfBuffer{harness_node->get_clock()};
        tf2_ros::TransformListener tfListener{tfBuffer};

        publish(msg::IK{}, "ik_pos_cmd");
        executor->spin_some();
        // arm controller should publish target position to tf tree
        SE3d target = SE3Conversions::fromTfTree(tfBuffer, "arm_target", "arm_base_link");
        EXPECT_EQ(target, SE3d::Identity());

        // there's nothing we can publicly check for velocity
        publish(geometry_msgs::msg::Twist{}, "ik_vel_cmd");
        executor->spin_some();

        msg::ControllerState fk_msg;
        fk_msg.names = {ArmController::JOINT_A, ArmController::JOINT_B, ArmController::JOINT_C, ArmController::JOINT_DE_PITCH, ArmController::JOINT_DE_ROLL, ArmController::JOINT_GRIPPER};
        // send 0 for all joint angles
        fk_msg.positions.resize(fk_msg.names.size());
        publish(fk_msg, "arm_controller_state");
        executor->spin_some();
        SE3d joint_a_pos = SE3Conversions::fromTfTree(tfBuffer, "arm_joint_a", "arm_base_link");
        EXPECT_EQ(joint_a_pos, SE3d::Identity());
        SE3d joint_b_pos = SE3Conversions::fromTfTree(tfBuffer, "arm_joint_b", "arm_joint_a");
        EXPECT_EQ(joint_b_pos, SE3d::Identity());
    }

    TEST_F(ArmControllerRosHarness, timerPosTest) {
        std::string arm_cmd_topic = "arm_pos_cmd";
        auto msg = subOne<msg::Position>(arm_cmd_topic, 50ms);
        EXPECT_FALSE(msg);

        std::string topic = "ik_pos_cmd";
        msg::IK ik_msg;
        // something unreachable
        ik_msg.pos.x = 10000;
        publish(ik_msg, topic);

        // shouldn't publish when we are out of bounds
        msg = subOne<msg::Position>(arm_cmd_topic, 50ms);
        EXPECT_FALSE(msg);

        // something reachable
        ik_msg.pos.x = 0.7;
        publish(ik_msg, topic);

        msg = subOne<msg::Position>(arm_cmd_topic, 50ms);
        ASSERT_TRUE(msg);
        EXPECT_FLOAT_EQ(msg->positions[0], 0);
    }

    TEST_F(ArmControllerRosHarness, timerVelTest) {
        // switch to velocity mode
        changeIkMode(srv::IkMode::Request::VELOCITY_CONTROL);

        std::string arm_cmd_topic = "arm_vel_cmd";
        auto msg = subOne<msg::Velocity>(arm_cmd_topic, 50ms);
        EXPECT_FALSE(msg);

        // send a velocity of 0 (nothing should be sent to prevent sag)
        std::string topic = "ik_vel_cmd";
        geometry_msgs::msg::Twist vel_msg;
        publish(vel_msg, topic);

        msg = subOne<msg::Velocity>(arm_cmd_topic, 50ms);
        EXPECT_FALSE(msg);

        // send a nonzero velocity, but it's not achievable
        vel_msg.linear.x = std::numeric_limits<double>::infinity();
        publish(vel_msg, topic);
        msg = subOne<msg::Velocity>(arm_cmd_topic, 50ms);
        EXPECT_FALSE(msg);

        // send a real velocity
        vel_msg.linear.x = 0.1;
        publish(vel_msg, topic);
        msg = subOne<msg::Velocity>(arm_cmd_topic, 50ms);
        EXPECT_TRUE(msg);
    }

    TEST_F(ArmControllerRosHarness, nominalActionTest) {
        // switch to typing mode
        changeIkMode(srv::IkMode::Request::TYPING);

        // we need to create a service to simulate the pusher
        auto pusher_serv = harness_node->create_service<srv::Pusher>("pusher", [](srv::Pusher::Request::ConstSharedPtr const&, srv::Pusher::Response::SharedPtr const& resp) -> void { resp->finished = true; });

        // send goal
        auto goal = action::TypingPosition::Goal();
        goal.x = 0.1; // something reasonable (this is joint a)
        const auto& goal_handle = sendGoalSync<action::TypingPosition>(goal, "typing_pos");
        // check that the goal was accepted
        ASSERT_TRUE(goal_handle);

        // let it cook for a bit
        executor->spin_all(50ms);

        // now update arm position so that the action can finish
        msg::ControllerState joint_msg;
        joint_msg.names = {ArmController::JOINT_A, ArmController::JOINT_B, ArmController::JOINT_C, ArmController::JOINT_DE_PITCH, ArmController::JOINT_DE_ROLL, ArmController::JOINT_GRIPPER};
        joint_msg.positions.resize(joint_msg.names.size());
        joint_msg.positions[0] = 0.1; // move joint a so that it's in the right spot
        publish(joint_msg, "arm_controller_state");

        // spin until the goal finishes
        const auto& result = getResult<action::TypingPosition>(goal_handle, "typing_pos", 5s);
        ASSERT_TRUE(result);
        EXPECT_EQ(result->code, rclcpp_action::ResultCode::SUCCEEDED);
        EXPECT_TRUE(result->result->success);
    }

    TEST_F(ArmControllerRosHarness, testActionCancel) {
        // switch to typing mode
        changeIkMode(srv::IkMode::Request::TYPING);

        auto goal = action::TypingPosition::Goal();
        goal.x = 0.2; // prevent action from finishing
        const auto& goal_handle = sendGoalSync<action::TypingPosition>(goal, "typing_pos");
        // check results
        ASSERT_TRUE(goal_handle);

        // cancel normally
        EXPECT_TRUE(cancelGoal<action::TypingPosition>(goal_handle, "typing_pos", 50ms));

        // spin until the goal cancels
        const auto& result = getResult<action::TypingPosition>(goal_handle, "typing_pos", 500ms);
        ASSERT_TRUE(result);
        EXPECT_EQ(result->code, rclcpp_action::ResultCode::CANCELED);
        EXPECT_FALSE(result->result->success);
    }

    TEST_F(ArmControllerRosHarness, actionTimeout) {
        // switch to typing mode
        changeIkMode(srv::IkMode::Request::TYPING);

        // send goal
        auto goal = action::TypingPosition::Goal();
        goal.x = 0.2; // something reasonable (this is joint a)
        const auto& goal_handle = sendGoalSync<action::TypingPosition>(goal, "typing_pos");
        // check that the goal was accepted
        ASSERT_TRUE(goal_handle);

        // spin until the goal finishes
        const auto& result = getResult<action::TypingPosition>(goal_handle, "typing_pos", 16s);
        ASSERT_TRUE(result);
        EXPECT_EQ(result->code, rclcpp_action::ResultCode::ABORTED);
        EXPECT_FALSE(result->result->success);
    }

    TEST_F(ArmControllerRosHarness, actionTestNoPusher) {
        // switch to typing mode
        changeIkMode(srv::IkMode::Request::TYPING);

        // send goal
        auto goal = action::TypingPosition::Goal();
        const auto& goal_handle = sendGoalSync<action::TypingPosition>(goal, "typing_pos");
        // check that the goal was accepted
        ASSERT_TRUE(goal_handle);

        // spin until the goal finishes
        const auto& result = getResult<action::TypingPosition>(goal_handle, "typing_pos", 5s);
        ASSERT_TRUE(result);
        EXPECT_EQ(result->code, rclcpp_action::ResultCode::SUCCEEDED);
        // should not suceed because no pusher
        EXPECT_FALSE(result->result->success);
    }

    TEST_F(ArmControllerRosHarness, actionTestBadPusher) {
        // switch to typing mode
        changeIkMode(srv::IkMode::Request::TYPING);

        // simulate pusher failure
        auto pusher_serv = harness_node->create_service<srv::Pusher>("pusher", [](srv::Pusher::Request::ConstSharedPtr const&, srv::Pusher::Response::SharedPtr const&) -> void { std::this_thread::sleep_for(6s); });

        // send goal
        auto goal = action::TypingPosition::Goal();
        const auto& goal_handle = sendGoalSync<action::TypingPosition>(goal, "typing_pos");
        // check that the goal was accepted
        ASSERT_TRUE(goal_handle);

        // spin until the goal finishes
        const auto& result = getResult<action::TypingPosition>(goal_handle, "typing_pos", 10s);
        ASSERT_TRUE(result);
        EXPECT_EQ(result->code, rclcpp_action::ResultCode::SUCCEEDED);
        // should not suceed because no pusher
        EXPECT_FALSE(result->result->success);
    }

    TEST_F(ArmControllerRosHarness, actionTestErrors) {
        auto goal = action::TypingPosition::Goal();
        // request should succeed, but server should reject because we are not
        // in typing mode
        auto goal_handle = sendGoalSync<action::TypingPosition>(goal, "typing_pos");
        EXPECT_FALSE(goal_handle);

        // switch to typing mode
        changeIkMode(srv::IkMode::Request::TYPING);

        // this time we should succeed
        goal.x = 100; // something unreachable
        goal_handle = sendGoalSync<action::TypingPosition>(goal, "typing_pos");
        ASSERT_TRUE(goal_handle);

        // if we ask again it should fail
        auto bad_goal_handle = sendGoalSync<action::TypingPosition>(goal, "typing_pos");
        EXPECT_FALSE(bad_goal_handle);

        // make sure nothing is published since we are out of bounds
        // spin for more than a timer cycle
        auto msg = subOne<msg::Position>("arm_pos_cmd", 50ms);
        EXPECT_FALSE(msg);

        // now update arm position so that the action can finish
        msg::ControllerState joint_msg;
        joint_msg.names = {ArmController::JOINT_A, ArmController::JOINT_B, ArmController::JOINT_C, ArmController::JOINT_DE_PITCH, ArmController::JOINT_DE_ROLL, ArmController::JOINT_GRIPPER};
        joint_msg.positions.resize(joint_msg.names.size());
        joint_msg.positions[0] = 100; // move joint a so that it's in the right spot
        publish(joint_msg, "arm_controller_state");

        // spin until the goal finishes - it should fail because no pusher
        const auto& result = getResult<action::TypingPosition>(goal_handle, "typing_pos", 5s);
        ASSERT_TRUE(result);
        EXPECT_EQ(result->code, rclcpp_action::ResultCode::SUCCEEDED);
        EXPECT_FALSE(result->result->success);
    }
}
