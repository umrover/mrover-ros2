#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "simulator.hpp"

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto simulator = std::make_shared<mrover::sim::Simulator>();
    executor.add_node(simulator);

    while (rclcpp::ok()) {
        executor.spin_some();
        simulator->step();

        auto period = std::chrono::duration<double>{1.0 / simulator->targetRate()};
        std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(period));
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
