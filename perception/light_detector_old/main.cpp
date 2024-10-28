#include "light_detector.hpp"
#include "pch.hpp"

// Load LightDetector nodelet if we are dynamically loading it
#ifdef MROVER_IS_NODELET

auto main(int argc, char* argv[]) -> int{
	// rclcpp::init(argc, argv, "light_detector");

	// nodelet::Loader loader;
    // loader.load(rclcpp::this_node::getName(), "mrover/LightDetector", ros::names::getRemappings(), {});

    // rclcpp::spin();

	// return EXIT_SUCCESS;

	rclcpp::init(argc, argv);

    // Create a component manager
    auto component_manager = std::make_shared<rclcpp_components::ComponentManager>(rclcpp::NodeOptions());
    
    // Load the LightDetector component
    auto component_name = "mrover::LightDetector";  
    auto node_name = "light_detector";
    auto remap_args = rclcpp::NodeOptions();  // Pass any remapping arguments here if needed

    component_manager->create_component_factory(component_name);
    component_manager->load_component(component_name, remap_args);

    // Spin with an executor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(component_manager);
    executor.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
