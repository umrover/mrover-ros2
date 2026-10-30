#include "harness.hpp"

namespace mrover {
    // TODO: think about what happens if we call this multiple times
    void ArmControllerRosHarness::changeIkMode(srv::IkMode_Request::_mode_type req) {
        auto mode_client = test_node->create_client<srv::IkMode>("ik_mode");
        ASSERT_TRUE(mode_client->wait_for_service(std::chrono::seconds(1)));

        auto request = std::make_shared<srv::IkMode::Request>();
        request->mode = req;

        auto response_future = mode_client->async_send_request(request);
        auto status = executor->spin_until_future_complete(response_future, std::chrono::milliseconds(50));
        ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
        auto resp = response_future.get();
        ASSERT_TRUE(resp->success);

    }

    TEST_F(ArmControllerRosHarness, topicTest) {
        const auto topic_map = arm_controller->get_topic_names_and_types();
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
        const auto service_map = arm_controller->get_service_names_and_types();
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
        tf2_ros::Buffer tfBuffer{test_node->get_clock()};
        tf2_ros::TransformListener tfListener{tfBuffer};

        auto pos_pub = test_node->create_publisher<msg::IK>("ik_pos_cmd", 10);
        msg::IK pos_msg;
        pos_pub->publish(pos_msg);
        executor->spin_some();
        // arm controller should publish target position to tf tree
        // TODO: consider making these checks only look at whether something is published and leave correctness for non-ros tests
        SE3d target = SE3Conversions::fromTfTree(tfBuffer, "arm_target", "arm_base_link");
        EXPECT_EQ(target, SE3d::Identity());

        // there's nothing we can publicly check for velocity
        auto vel_pub = test_node->create_publisher<geometry_msgs::msg::Twist>("ik_vel_cmd", 10);
        geometry_msgs::msg::Twist vel_msg;
        vel_pub->publish(vel_msg);
        executor->spin_some();

        auto joint_pub = test_node->create_publisher<msg::ControllerState>("arm_controller_state", 10);
        msg::ControllerState fk_msg;
        fk_msg.names = {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll", "gripper"};
        // send 0 for all joint angles
        fk_msg.positions.resize(fk_msg.names.size());
        joint_pub->publish(fk_msg);
        executor->spin_some();
        SE3d joint_a_pos = SE3Conversions::fromTfTree(tfBuffer, "arm_joint_a", "arm_base_link");
        EXPECT_EQ(joint_a_pos, SE3d::Identity());
        SE3d joint_b_pos = SE3Conversions::fromTfTree(tfBuffer, "arm_joint_b", "arm_joint_a");
        EXPECT_EQ(joint_b_pos, SE3d::Identity());
    }

    TEST_F(ArmControllerRosHarness, timerPosTest) {
        auto msg_promise = std::make_shared<std::promise<msg::Position>>();
        std::future<msg::Position> msg_future = msg_promise->get_future();
        auto pos_sub = test_node->create_subscription<msg::Position>("arm_pos_cmd", 1, [msg_promise](msg::Position::ConstSharedPtr const& msg) -> void {
                msg_promise->set_value(*msg);
        });

        // since we haven't sent a command, the arm controller shouldn't
        // publish anything
        auto status = executor->spin_until_future_complete(msg_future, std::chrono::milliseconds(50));
        EXPECT_EQ(status, rclcpp::FutureReturnCode::TIMEOUT);

        auto ik_pub = test_node->create_publisher<msg::IK>("ik_pos_cmd", 10);
        msg::IK ik_msg;
        // something unreachable
        ik_msg.pos.x = 10000;
        ik_pub->publish(ik_msg);

        // shouldn't publish when we are out of bounds
        status = executor->spin_until_future_complete(msg_future, std::chrono::milliseconds(50));
        EXPECT_EQ(status, rclcpp::FutureReturnCode::TIMEOUT);

        // something reachable
        ik_msg.pos.x = 0.7;
        ik_pub->publish(ik_msg);

        status = executor->spin_until_future_complete(msg_future, std::chrono::milliseconds(50));
        EXPECT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
        EXPECT_FLOAT_EQ(msg_future.get().positions[0], 0);
    }

    TEST_F(ArmControllerRosHarness, timerVelTest) {
        // switch to velocity mode
        changeIkMode(srv::IkMode::Request::VELOCITY_CONTROL);

        auto msg_promise = std::make_shared<std::promise<msg::Velocity>>();
        std::future<msg::Velocity> msg_future = msg_promise->get_future();
        auto pos_sub = test_node->create_subscription<msg::Velocity>("arm_vel_cmd", 1, [msg_promise](msg::Velocity::ConstSharedPtr const& msg) -> void {
                msg_promise->set_value(*msg);
        });

        // haven't sent any commands yet so nothing should be published
        auto status = executor->spin_until_future_complete(msg_future, std::chrono::milliseconds(50));
        EXPECT_EQ(status, rclcpp::FutureReturnCode::TIMEOUT);

        // send a velocity of 0 (nothing should be sent to prevent sag)
        auto ik_vel_pub = test_node->create_publisher<geometry_msgs::msg::Twist>("ik_vel_cmd", 10);
        geometry_msgs::msg::Twist vel_msg;
        ik_vel_pub->publish(vel_msg);

        status = executor->spin_until_future_complete(msg_future, std::chrono::milliseconds(50));
        EXPECT_EQ(status, rclcpp::FutureReturnCode::TIMEOUT);

        // send a nonzero velocity, but it's not achievable
        vel_msg.linear.x = std::numeric_limits<double>::infinity();
        ik_vel_pub->publish(vel_msg);
        status = executor->spin_until_future_complete(msg_future, std::chrono::milliseconds(50));
        EXPECT_EQ(status, rclcpp::FutureReturnCode::TIMEOUT);

        // send a real velocity
        vel_msg.linear.x = 0.1;
        ik_vel_pub->publish(vel_msg);
        status = executor->spin_until_future_complete(msg_future, std::chrono::milliseconds(50));
        EXPECT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    }

    TEST_F(ArmControllerRosHarness, nominalActionTest) {
        auto action_client = rclcpp_action::create_client<action::TypingPosition>(test_node, "typing_pos");
        ASSERT_TRUE(action_client->wait_for_action_server());

        // switch to typing mode
        changeIkMode(srv::IkMode::Request::TYPING);

        // we need to create a service to simulate the pusher
        auto pusher_serv = test_node->create_service<srv::Pusher>("pusher", [](srv::Pusher::Request::ConstSharedPtr const&, srv::Pusher::Response::SharedPtr const& resp) -> void { resp->finished = true; });

        // send goal
        auto goal = action::TypingPosition::Goal();
        goal.x = 0.1; // something reasonable (this is joint a)
        auto goal_handle_future = action_client->async_send_goal(goal);
        auto status = executor->spin_until_future_complete(goal_handle_future, std::chrono::milliseconds(50));
        ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);

        // check that the goal was accepted
        const auto& goal_handle = goal_handle_future.get();
        EXPECT_TRUE(goal_handle);

        // let it cook for a bit
        // TODO: should probably check that commands are actually published
        status = executor->spin_until_future_complete(std::promise<void>().get_future(), std::chrono::milliseconds(50));
        EXPECT_EQ(status, rclcpp::FutureReturnCode::TIMEOUT);

        // now update arm position so that the action can finish
        auto joint_pub = test_node->create_publisher<msg::ControllerState>("arm_controller_state", 10);
        msg::ControllerState joint_msg;
        joint_msg.names = {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll", "gripper"};
        joint_msg.positions.resize(joint_msg.names.size());
        joint_msg.positions[0] = 0.1; // move joint a so that it's in the right spot
        joint_pub->publish(joint_msg);

        // spin until the goal finishes
        auto result_future = action_client->async_get_result(goal_handle);
        status = executor->spin_until_future_complete(result_future, std::chrono::milliseconds(5000));
        ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
        const auto& result = result_future.get();
        EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
        EXPECT_TRUE(result.result->success);
    }

    TEST_F(ArmControllerRosHarness, testActionCancel) {
        auto action_client = rclcpp_action::create_client<action::TypingPosition>(test_node, "typing_pos");
        ASSERT_TRUE(action_client->wait_for_action_server());

        // switch to typing mode
        changeIkMode(srv::IkMode::Request::TYPING);

        auto goal = action::TypingPosition::Goal();
        goal.x = 0.2; // prevent action from finishing
        auto goal_handle_future = action_client->async_send_goal(goal);

        auto status = executor->spin_until_future_complete(goal_handle_future, std::chrono::milliseconds(50));
        ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);

        // check results
        const auto& goal_handle = goal_handle_future.get();
        EXPECT_TRUE(goal_handle);

        // cancel normally
        auto cancel_future = action_client->async_cancel_goal(goal_handle);

        status = executor->spin_until_future_complete(cancel_future, std::chrono::milliseconds(50));
        ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
        EXPECT_TRUE(cancel_future.get());

        // spin until the goal cancels
        auto result_future = action_client->async_get_result(goal_handle);
        status = executor->spin_until_future_complete(result_future, std::chrono::milliseconds(500));
        ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
        const auto& result = result_future.get();
        EXPECT_EQ(result.code, rclcpp_action::ResultCode::CANCELED);
        EXPECT_FALSE(result.result->success);
    }

    TEST_F(ArmControllerRosHarness, actionTimeout) {
        auto action_client = rclcpp_action::create_client<action::TypingPosition>(test_node, "typing_pos");
        ASSERT_TRUE(action_client->wait_for_action_server());

        // switch to typing mode
        changeIkMode(srv::IkMode::Request::TYPING);

        // send goal
        auto goal = action::TypingPosition::Goal();
        goal.x = 0.2; // something reasonable (this is joint a)
        auto goal_handle_future = action_client->async_send_goal(goal);
        auto status = executor->spin_until_future_complete(goal_handle_future, std::chrono::milliseconds(50));
        ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);

        // check that the goal was accepted
        const auto& goal_handle = goal_handle_future.get();
        EXPECT_TRUE(goal_handle);

        // spin until the goal finishes
        auto result_future = action_client->async_get_result(goal_handle);
        status = executor->spin_until_future_complete(result_future, std::chrono::milliseconds(16000));
        ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
        const auto& result = result_future.get();
        EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
        EXPECT_FALSE(result.result->success);
    }

    TEST_F(ArmControllerRosHarness, actionTestNoPusher) {
        auto action_client = rclcpp_action::create_client<action::TypingPosition>(test_node, "typing_pos");
        ASSERT_TRUE(action_client->wait_for_action_server());

        // switch to typing mode
        changeIkMode(srv::IkMode::Request::TYPING);

        // send goal
        auto goal = action::TypingPosition::Goal();
        auto goal_handle_future = action_client->async_send_goal(goal);
        auto status = executor->spin_until_future_complete(goal_handle_future, std::chrono::milliseconds(50));
        ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);

        // check that the goal was accepted
        const auto& goal_handle = goal_handle_future.get();
        EXPECT_TRUE(goal_handle);

        // spin until the goal finishes
        auto result_future = action_client->async_get_result(goal_handle);
        status = executor->spin_until_future_complete(result_future, std::chrono::milliseconds(5000));
        ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
        const auto& result = result_future.get();
        EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
        // should not suceed because no pusher
        EXPECT_FALSE(result.result->success);
    }

    TEST_F(ArmControllerRosHarness, actionTestBadPusher) {
        auto action_client = rclcpp_action::create_client<action::TypingPosition>(test_node, "typing_pos");
        ASSERT_TRUE(action_client->wait_for_action_server());

        // switch to typing mode
        changeIkMode(srv::IkMode::Request::TYPING);

        // simulate pusher failure
        auto pusher_serv = test_node->create_service<srv::Pusher>("pusher", [](srv::Pusher::Request::ConstSharedPtr const&, srv::Pusher::Response::SharedPtr const&) -> void { std::this_thread::sleep_for(std::chrono::seconds(6)); });

        // send goal
        auto goal = action::TypingPosition::Goal();
        auto goal_handle_future = action_client->async_send_goal(goal);
        auto status = executor->spin_until_future_complete(goal_handle_future, std::chrono::milliseconds(50));
        ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);

        // check that the goal was accepted
        const auto& goal_handle = goal_handle_future.get();
        EXPECT_TRUE(goal_handle);

        // spin until the goal finishes
        auto result_future = action_client->async_get_result(goal_handle);
        status = executor->spin_until_future_complete(result_future, std::chrono::milliseconds(10000));
        ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
        const auto& result = result_future.get();
        EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
        // should not suceed because no pusher
        EXPECT_FALSE(result.result->success);
    }

    TEST_F(ArmControllerRosHarness, actionTestErrors) {
        auto action_client = rclcpp_action::create_client<action::TypingPosition>(test_node, "typing_pos");
        ASSERT_TRUE(action_client->wait_for_action_server());

        auto goal = action::TypingPosition::Goal();
        // future should succeed, but server should reject because we are not
        // in typing mode
        auto goal_handle_future = action_client->async_send_goal(goal);
        auto status = executor->spin_until_future_complete(goal_handle_future, std::chrono::milliseconds(50));
        ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
        auto goal_handle = goal_handle_future.get();
        EXPECT_FALSE(goal_handle);

        // switch to typing mode
        changeIkMode(srv::IkMode::Request::TYPING);

        // this time we should succeed
        goal.x = 100; // something unreachable
        rclcpp_action::Client<action::TypingPosition>::SendGoalOptions opts;
        auto done_promise = std::make_shared<std::promise<void>>();
        auto done_future = done_promise->get_future();
        opts.result_callback = [done_promise](auto) -> void { done_promise->set_value(); };
        goal_handle_future = action_client->async_send_goal(goal, opts);

        // if we ask again it should fail
        auto bad_goal_handle_future = action_client->async_send_goal(goal);

        // complete both futures
        status = executor->spin_until_future_complete(goal_handle_future, std::chrono::milliseconds(50));
        ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
        status = executor->spin_until_future_complete(bad_goal_handle_future, std::chrono::milliseconds(50));
        ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);

        // check results
        goal_handle = goal_handle_future.get();
        EXPECT_TRUE(goal_handle);
        EXPECT_FALSE(bad_goal_handle_future.get());

        // make sure nothing is published since we are out of bounds
        auto msg_promise = std::promise<msg::Position>();
        auto func = [&msg_promise](msg::Position::ConstSharedPtr const& msg) -> void { msg_promise.set_value(*msg); };
        func(std::make_shared<msg::Position>()); // do this to satisfy coverage checks
        msg_promise = std::promise<msg::Position>(); // reset the promise
        auto pos_sub = test_node->create_subscription<msg::Position>("arm_pos_cmd", 1, func);
        // spin for more than a timer cycle
        status = executor->spin_until_future_complete(msg_promise.get_future(), std::chrono::milliseconds(50));
        EXPECT_EQ(status, rclcpp::FutureReturnCode::TIMEOUT);

        // now update arm position so that the action can finish
        auto joint_pub = test_node->create_publisher<msg::ControllerState>("arm_controller_state", 10);
        msg::ControllerState joint_msg;
        joint_msg.names = {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll", "gripper"};
        joint_msg.positions.resize(joint_msg.names.size());
        joint_msg.positions[0] = 100; // move joint a so that it's in the right spot
        joint_pub->publish(joint_msg);

        // spin until the goal finishes - it should fail because no pusher
        auto result_future = action_client->async_get_result(goal_handle);
        status = executor->spin_until_future_complete(result_future, std::chrono::milliseconds(5000));
        ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
        const auto& result = result_future.get();
        EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
        EXPECT_FALSE(result.result->success);
        // // actually check that we got something nonzero
        // EXPECT_FLOAT_EQ(pos_cmd.positions[0], 0.1);
        //
        // // wait until the actual worker thread finishes
        // status = executor->spin_until_future_complete(done_future, std::chrono::milliseconds(500));
        // ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    }
}
