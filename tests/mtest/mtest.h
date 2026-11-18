#pragma once
#include <gtest/gtest.h>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>

namespace mrover {
	template<class node_t>
	class NodeHarness : public ::testing::Test {
	protected:
		std::shared_ptr<node_t> node;

		void SetUp() override {
			node = std::make_shared<node_t>();
		}

		static void SetUpTestSuite() {
			rclcpp::init(0, nullptr);
		}

		static void TearDownTestSuite() {
			rclcpp::shutdown();
		}
	};

	template<class harness_t>
	class NodeRosHarness : public harness_t {
	protected:
		using harness_t::node;
		std::shared_ptr<rclcpp::Node> harness_node;
		std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;

		void SetUp() override {
			harness_t::SetUp();
			harness_node = std::make_shared<rclcpp::Node>("harness_node");
			executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

			executor->add_node(harness_node);
			executor->add_node(node);
		}
	};
}
