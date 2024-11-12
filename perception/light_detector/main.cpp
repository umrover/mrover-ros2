#include "light_detector.hpp"

auto main(int argc, char* argv[]) -> int{
    rclcpp::init(argc, argv);

    // DO NOT REMOVE OR ELSE REF COUNT WILL GO TO ZERO
    auto lightOD = std::make_shared<mrover::LightDetector>();
    std::cout<<"in main"<<std::endl;
    rclcpp::executors::SingleThreadedExecutor executor;

    executor.add_node(lightOD);
    std::cout<<"add node to executor"<<std::endl;

    executor.spin();
    std::cout<<"spun executor"<<std::endl;


    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
