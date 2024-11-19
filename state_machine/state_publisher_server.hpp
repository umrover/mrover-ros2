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

        void publishStructure(){
            auto structureMsg = mrover::msg::StateMachineStructure();
            structureMsg.machine_name = mStateMachine.getName();
			auto transitionTable = mStateMachine.getTransitionTable();

			for(auto const&[from, tos] : transitionTable){
				auto transition = mrover::msg::StateMachineTransition();
				transition.origin = mStateMachine.decodeTypeHash(from);
				for(auto& hash : tos){
					std::cout << mStateMachine.decodeTypeHash(hash) << std::endl;
					transition.destinations.push_back(mStateMachine.decodeTypeHash(hash));
				}
				structureMsg.transitions.push_back(std::move(transition));
			}

			mStructurePub->publish(structureMsg);
        }

        void publishState(){
			auto stateMachineUpdate = mrover::msg::StateMachineStateUpdate();
			stateMachineUpdate.state_machine_name = mStateMachine.getName();
			stateMachineUpdate.state = mStateMachine.getCurrentState();
			mStatePub->publish(stateMachineUpdate);
        }

    public:
        StatePublisher(rclcpp::Node* node, StateMachine const& stateMachine, std::string const& structureTopicName, double structureTopicHz, std::string const& stateTopicName, double stateTopicHz) : mStateMachine{stateMachine} {
            mStructurePub = node->create_publisher<mrover::msg::StateMachineStructure>(structureTopicName, 1);
            mStatePub = node->create_publisher<mrover::msg::StateMachineStateUpdate>(stateTopicName, 1);

            mStructureTimer = node->create_wall_timer(std::chrono::milliseconds(static_cast<std::size_t>(1 / structureTopicHz)), [&](){publishStructure();});
            mStateTimer = node->create_wall_timer(std::chrono::milliseconds(static_cast<std::size_t>(1 / stateTopicHz)), [&](){publishState();});
        }
    };
}
/*
from mrover.msg import StateMachineStructure, StateMachineTransition, StateMachineStateUpdate
from rclpy.node import Node
from rclpy.publisher import Publisher
from state_machine.state_machine import StateMachine

class StatePublisher:
    structure_publisher: Publisher
    state_publisher: Publisher
    state_machine: StateMachine

    def __init__(
        self,
        node: Node,
        state_machine: StateMachine,
        structure_pub_topic: str,
        structure_update_rate_hz: float,
        state_pub_topic: str,
        state_update_rate_hz: float,
    ):
        self.state_machine = state_machine
        self.structure_publisher = node.create_publisher(StateMachineStructure, structure_pub_topic, 1)
        self.state_publisher = node.create_publisher(StateMachineStateUpdate, state_pub_topic, 1)
        node.create_timer(1 / structure_update_rate_hz, self.publish_structure)
        node.create_timer(1 / state_update_rate_hz, self.publish_state)

    def publish_structure(self) -> None:
        structure = StateMachineStructure()
        structure.machine_name = self.state_machine.name
        for origin, destinations in self.state_machine.state_transitions.items():
            transition = StateMachineTransition()
            transition.origin = origin.__name__
            transition.destinations = [dest.__name__ for dest in destinations]
            structure.transitions.append(transition)
        self.structure_publisher.publish(structure)

    def publish_state(self) -> None:
        current_state = self.state_machine.current_state
        state = StateMachineStateUpdate(state=str(current_state), state_machine_name=self.state_machine.name)
        self.state_publisher.publish(state)
*/
