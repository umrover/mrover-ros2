#include "simulator.hpp"

namespace mrover {

    Simulator::Simulator() : Node{"simulator", rclcpp::NodeOptions{}
                                                       .use_intra_process_comms(true)
                                                       .allow_undeclared_parameters(true)
                                                       .automatically_declare_parameters_from_overrides(true)} {
        try {
            mSaveTask = PeriodicTask{get_parameter("save_rate").as_double()};
            mSaveHistory = boost::circular_buffer<SaveData>{static_cast<std::size_t>(get_parameter("save_history").as_int())};

            mGroundTruthPub = create_publisher<nav_msgs::msg::Odometry>("ground_truth", 1);

            mCmdVelPub = create_publisher<geometry_msgs::msg::Twist>("sim_cmd_vel", 1);

            mImageTargetsPub = create_publisher<msg::ImageTargets>("objects", 1);

            mIkTargetPub = create_publisher<msg::IK>("arm_ik", 1);

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

            initUrdfsFromParams();

            {
                auto addGroup = [&](std::string_view groupName, std::vector<std::string> const& names) {
                    MotorGroup& group = mMotorGroups.emplace_back();
                    group.jointStatePub = create_publisher<sensor_msgs::msg::JointState>(std::format("{}_joint_data", groupName), 1);
                    group.controllerStatePub = create_publisher<msg::ControllerState>(std::format("{}_controller_data", groupName), 1);
                    group.names = names;
                    group.throttleSub = create_subscription<msg::Throttle>(std::format("{}_throttle_cmd", groupName), 1, [this](msg::Throttle::SharedPtr msg) {
                        throttlesCallback(msg);
                    });
                    group.velocitySub = create_subscription<msg::Velocity>(std::format("{}_velocity_cmd", groupName), 1, [this](msg::Velocity::SharedPtr msg) {
                        velocitiesCallback(msg);
                    });
                    group.positionSub = create_subscription<msg::Position>(std::format("{}_position_cmd", groupName), 1, [this](msg::Position::SharedPtr msg) {
                        positionsCallback(msg);
                    });
                };
                addGroup("arm", {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll"});
                addGroup("drive_left", {"front_left", "middle_left", "back_left"});
                addGroup("drive_right", {"front_right", "middle_right", "back_right"});
            }

            mRunThread = std::thread{&Simulator::run, this};

        } catch (std::exception const& e) {
            RCLCPP_FATAL_STREAM(get_logger(), e.what());
            rclcpp::shutdown();
        }
    }

    auto spinSleep(Clock::time_point const& until) -> void {
        //  See: https://blat-blatnik.github.io/computerBear/making-accurate-sleep-function/
        // Idea: sleep until 1ms before the desired time, then spin until the desired time
        std::this_thread::sleep_until(until - 1ms);
        while (Clock::now() < until)
            ;
    }

    auto Simulator::run() -> void try {
        for (Clock::duration dt{}; rclcpp::ok();) {
            Clock::time_point beginTime = Clock::now();

            mLoopProfiler.beginLoop();

            // Note(quintin):
            // Apple sucks and does not support polling on a non-main thread (which we are currently on)
            // Instead add to ROS's global callback queue which I think will be served by the main thread
            if (!mIsHeadless) {
#ifdef __APPLE__
                struct PollGlfw : ros::CallbackInterface {
                    auto call() -> CallResult override {
                        glfwPollEvents();
                        return Success;
                    }
                };
                ros::getGlobalCallbackQueue()->addCallback(boost::make_shared<PollGlfw>());
#else
                glfwPollEvents();
#endif
                // Comments this out while debugging segfaults, otherwise it captures your cursor
                // glfwSetInputMode(mWindow.get(), GLFW_CURSOR, mInGui ? GLFW_CURSOR_NORMAL : GLFW_CURSOR_DISABLED);
            }
            mLoopProfiler.measureEvent("GLFW Events");

            userControls(dt);
            mLoopProfiler.measureEvent("Controls");

            if (mEnablePhysics) physicsUpdate(dt);
            mLoopProfiler.measureEvent("Physics");

            renderUpdate();
            mLoopProfiler.measureEvent("Render");

            spinSleep(beginTime + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<float>{1.0f / mTargetUpdateRate}));
            dt = Clock::now() - beginTime;

            mLoopProfiler.measureEvent("Sleep");
        }

    } catch (std::exception const& e) {
        RCLCPP_FATAL_STREAM(get_logger(), e.what());
        rclcpp::shutdown();
    }

    Simulator::~Simulator() {
        mRunThread.join();

        if (!mIsHeadless) {
            ImGui_ImplWGPU_Shutdown();
            ImGui_ImplGlfw_Shutdown();
            ImGui::DestroyContext();
        }

        for (int i = mDynamicsWorld->getNumConstraints() - 1; i >= 0; --i) {
            mDynamicsWorld->removeConstraint(mDynamicsWorld->getConstraint(i));
        }

        for (int i = mDynamicsWorld->getNumCollisionObjects() - 1; i >= 0; --i) {
            btCollisionObject* object = mDynamicsWorld->getCollisionObjectArray()[i];
            mDynamicsWorld->removeCollisionObject(object);
        }
    }

} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::Simulator>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}