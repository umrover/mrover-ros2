#include "simulator.hpp"

namespace mrover {

    Simulator::Simulator() : Node{"simulator", rclcpp::NodeOptions{}
                                                       .use_intra_process_comms(true)
                                                       .allow_undeclared_parameters(true)
                                                       .automatically_declare_parameters_from_overrides(true)} {
        double saveRate;
        int saveHistory;
        int motorTimeoutMs;
        std::vector<ParameterWrapper> params = {
                {"save_rate", saveRate, 1.0},
                {"save_history", saveHistory, 4096},
                {"motor_timeout", motorTimeoutMs, 100},
                {"headless", mIsHeadless, false},
                {"ref_lat", mGpsLinearizationReferencePoint(0), 42.293195},
                {"ref_lon", mGpsLinearizationReferencePoint(1), -83.7096706},
                {"ref_alt", mGpsLinearizationReferencePoint(2), 0.0},
                {"ref_heading", mGpsLinerizationReferenceHeading, 90.0}};
        ParameterWrapper::declareParameters(this, params);

        try {
            mSaveTask = PeriodicTask{saveRate};
            mSaveHistory = boost::circular_buffer<SaveData>{static_cast<std::size_t>(saveHistory)};
            mMotorTimeoutMs = motorTimeoutMs;
            mEnablePhysics = mIsHeadless;

            mGroundTruthPub = create_publisher<nav_msgs::msg::Odometry>("ground_truth", 1);

            mCmdVelPub = create_publisher<geometry_msgs::msg::Twist>("sim_cmd_vel", 1);

            mImageTargetsPub = create_publisher<msg::ImageTargets>("objects", 1);

            mIkTargetPub = create_publisher<msg::IK>("ee_pos_cmd", 1);

            mIkVelPub = create_publisher<geometry_msgs::msg::Twist>("ee_vel_cmd", 1);

            mIkModeClient = create_client<srv::IkMode>("ik_mode");

            mActionClient = rclcpp_action::create_client<action::ClickIk>(this->get_node_base_interface(),
                                                                          this->get_node_graph_interface(),
                                                                          this->get_node_logging_interface(),
                                                                          this->get_node_waitables_interface(),
                                                                          "click_ik");

            mMotorTimeoutMs = get_parameter("motor_timeout").as_int();

            mIsHeadless = get_parameter("headless").as_bool();
            mEnablePhysics = mIsHeadless;
            {
                mGpsLinearizationReferencePoint = {
                        get_parameter("ref_lat").as_double(),
                        get_parameter("ref_lon").as_double(),
                        get_parameter("ref_alt").as_double(),
                };
                mGpsLinerizationReferenceHeading = get_parameter("ref_heading").as_double();
            }

            if (!mIsHeadless) initWindow();

            initPhysics();

            initRender();

            initUrdfsFromParams(DEFAULT_MAP);

            {
                auto addGroup = [&](std::string_view groupName, std::vector<std::string> const& names) {
                    MotorGroup& group = mMotorGroups.emplace_back();
                    group.jointStatePub = create_publisher<sensor_msgs::msg::JointState>(std::format("{}_joint_data", groupName), 1);
                    group.controllerStatePub = create_publisher<msg::ControllerState>(std::format("{}_controller_data", groupName), 1);
                    group.names = names;
                    group.throttleSub = create_subscription<msg::Throttle>(std::format("{}_throttle_cmd", groupName), 1, [this](msg::Throttle::ConstSharedPtr const& msg) {
                        throttlesCallback(msg);
                    });
                    group.velocitySub = create_subscription<msg::Velocity>(std::format("{}_velocity_cmd", groupName), 1, [this](msg::Velocity::ConstSharedPtr const& msg) {
                        velocitiesCallback(msg);
                    });
                    group.positionSub = create_subscription<msg::Position>(std::format("{}_position_cmd", groupName), 1, [this](msg::Position::ConstSharedPtr const& msg) {
                        positionsCallback(msg);
                    });
                };
                addGroup("arm", {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll", "gripper"});
                addGroup("arm", {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll", "gripper"});
                addGroup("drive_left", {"front_left", "middle_left", "back_left"});
                addGroup("drive_right", {"front_right", "middle_right", "back_right"});

                std::vector<decltype(mMsgToUrdf)::value_type> elements{
                        {"joint_a", "arm_a_link"},
                        {"joint_b", "arm_b_link"},
                        {"joint_c", "arm_c_link"},
                        {"joint_de_pitch", "arm_d_link"},
                        {"joint_de_roll", "arm_e_link"},
                        {"gripper", "arm_gripper_link"},
                        {"gripper", "arm_gripper_link"},
                        {"front_left", "front_left_wheel_link"},
                        {"middle_left", "center_left_wheel_link"},
                        {"back_left", "back_left_wheel_link"},
                        {"front_right", "front_right_wheel_link"},
                        {"middle_right", "center_right_wheel_link"},
                        {"back_right", "back_right_wheel_link"},
                        {"mast_gimbal_y", "zed_mini_camera"},
                };
                mMsgToUrdf.insert(elements.begin(), elements.end());
            }
        } catch (std::exception const& e) {
            RCLCPP_FATAL_STREAM(get_logger(), e.what());
            rclcpp::shutdown();
        }
    }

    auto spinSleep(Clock::time_point const& until) -> void {
        //  See: https://blat-blatnik.github.io/computerBear/making-accurate-sleep-function/
        // Idea: sleep until 1ms before the desired time, then spin until the desired time
        std::this_thread::sleep_until(until - 1ms);
        while (Clock::now() < until);
    }

    auto Simulator::tick() -> void try {
        Clock::time_point begin = Clock::now();
        Clock::duration dt = begin - mLastTickTime;

        mLoopProfiler.beginLoop();

        if (!mIsHeadless) {
            glfwPollEvents();
            // Comments this out while debugging segfaults, otherwise it captures your cursor
            glfwSetInputMode(mWindow.get(), GLFW_CURSOR, mInGui ? GLFW_CURSOR_NORMAL : GLFW_CURSOR_DISABLED);
        }
        mLoopProfiler.measureEvent("GLFW Events");

        userControls(dt);
        mLoopProfiler.measureEvent("Controls");

        if (mEnablePhysics) physicsUpdate(dt);
        mLoopProfiler.measureEvent("Physics");

        renderUpdate();
        mLoopProfiler.measureEvent("Render");

        spinSleep(begin + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<float>{1.0f / mTargetUpdateRate}));
        mLoopProfiler.measureEvent("Sleep");

        mLastTickTime = begin;

    } catch (std::exception const& e) {
        RCLCPP_FATAL_STREAM(get_logger(), e.what());
        rclcpp::shutdown();
    }

    Simulator::~Simulator() {
        if (!mIsHeadless) {
            ImGui_ImplWGPU_Shutdown();
            ImGui_ImplGlfw_Shutdown();
            ImGui::DestroyContext();
        }
    }

} // namespace mrover


// #include "rclcpp_components/register_node_macro.hpp"
// RCLCPP_COMPONENTS_REGISTER_NODE(mrover::Simulator)

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto simulator = std::make_shared<mrover::Simulator>();
    executor.add_node(simulator);
    while (rclcpp::ok()) {
        executor.spin_some();
        simulator->tick();
    }
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
