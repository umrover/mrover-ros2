#pragma once

// STL
#include <cstddef>
#include <memory>

// Ros Client Library
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// MRover
#include "mrover/msg/detail/state_machine_state_update__struct.hpp"
#include "mrover/msg/detail/state_machine_structure__struct.hpp"
#include "state_machine.hpp"
#include <mrover/msg/state_machine_structure.hpp>
#include <mrover/msg/state_machine_state_update.hpp>

namespace mrover{
	class StatePublisher{
	private:
		StateMachine const& mStateMachine;

		rclcpp::Publisher<mrover::msg::StateMachineStructure>::SharedPtr mStructurePub;
		rclcpp::Publisher<mrover::msg::StateMachineStateUpdate>::SharedPtr mStatePub;

		rclcpp::TimerBase::SharedPtr mStructureTimer;
		rclcpp::TimerBase::SharedPtr mStateTimer;
		/**
		 * \brief             Publishes the structure to be used by visualizer.py
		 * \see               visualizer.py to see how these topic will be used
		 */
		void publishStructure(){
			auto structureMsg = mrover::msg::StateMachineStructure();
			structureMsg.machine_name = mStateMachine.getName();
			auto transitionTable = mStateMachine.getTransitionTable();

			for(auto const&[from, tos] : transitionTable){
				auto transition = mrover::msg::StateMachineTransition();
				transition.origin = mStateMachine.decodeTypeHash(from);
				for(auto& hash : tos){
					transition.destinations.push_back(mStateMachine.decodeTypeHash(hash));
				}
				structureMsg.transitions.push_back(std::move(transition));
			}

			mStructurePub->publish(structureMsg);
		}

		/**
		 * \brief             Publishes the current state of the state machine
		 * \see               visualizer.py to see how these topic will be used
		 */
		void publishState(){
			auto stateMachineUpdate = mrover::msg::StateMachineStateUpdate();
			stateMachineUpdate.state_machine_name = mStateMachine.getName();
			stateMachineUpdate.state = mStateMachine.getCurrentStateName();
			mStatePub->publish(stateMachineUpdate);
		}

	public:
		/**
		 * \brief                       Creates a State Publisher to facilitate the communications between visualizer.py and the state machine
		 * \param node		            The node which owns the state publisher
		 * \param stateMachine          The state machine which the publisher will describe
		 * \param structureTopicName    The topic which will publish the state machine's structure
		 * \param structureTopicHz      The rate at which the structure topic will publish
		 * \param stateTopicName        The topic which will publish the state machine's state
		 * \param stateTopicHz          The rate at which the state topic will publish
		 */
		StatePublisher(rclcpp::Node* node, StateMachine const& stateMachine, std::string const& structureTopicName, double structureTopicHz, std::string const& stateTopicName, double stateTopicHz) : mStateMachine{stateMachine} {
			mStructurePub = node->create_publisher<mrover::msg::StateMachineStructure>(structureTopicName, 1);
			mStatePub = node->create_publisher<mrover::msg::StateMachineStateUpdate>(stateTopicName, 1);

			mStructureTimer = node->create_wall_timer(std::chrono::milliseconds(static_cast<std::size_t>(1000 / structureTopicHz)), [&](){publishStructure();});
			mStateTimer = node->create_wall_timer(std::chrono::milliseconds(static_cast<std::size_t>(1000 / stateTopicHz)), [&](){publishState();});
		}
	};
}
