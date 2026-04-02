#include "arm_controller.hpp"
#include "mrover/msg/detail/position__struct.hpp"
#include <chrono>
#include <cstdlib>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <rclcpp/timer.hpp>


namespace mrover {

    void publishFakeData();

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time mLastUpdate;



    ArmController::ArmController() : Node{"joint_gripper"}, mLastUpdate{get_clock()->now()} {
        jointA_Pub = create_publisher<msg::Position>("joint_a", 10);
        gripperPub = create_publisher<msg::Position>("gripper", 10);
        armPub = create_publisher<msg::Position>("arm_position_cmd",10);

        timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
    publishFakeData();
});

        
    }

    
    void ArmController::publishFakeData(){
        mrover::msg::Position joint_a;
        joint_a.names = {"joint_a"};
        joint_a.positions = {static_cast<float>(std::rand())/RAND_MAX};

        mrover::msg::Position gripper;
        gripper.names = {"gripper"};
        gripper.positions = {static_cast<float>(std::rand()) / RAND_MAX};


        jointA_Pub->publish(joint_a);
        gripperPub->publish(gripper);

        RCLCPP_INFO(get_logger(),"joint_a=%.2f,gripper=%.2f",joint_a.positions[0]-joints["joint_a"].pos,gripper.positions[0]-joints["gripper"].pos);

        mrover::msg::Position pub_combined;
        pub_combined.names = {"joint_a","gripper"};
        pub_combined.positions = {joint_a.positions[0],gripper.positions[0]};

        armPub->publish(pub_combined);
    }

}

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::ArmController>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}