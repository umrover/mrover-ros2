#pragma once
#include "state.hpp"

#include <unordered_map>
#include <vector>
#include <typeinfo>
#include <stdexcept>
#include <format>
#include <algorithm>

class StateMachine{
private:
	State* currState;
	std::unordered_map<std::size_t, std::vector<std::size_t>> validTransitions;

	void assertValidTransition(std::type_info const& from, std::type_info const& to) const {
		auto it = validTransitions.find(from.hash_code());
		if(it == validTransitions.end()){
			throw std::runtime_error(std::format("{} is not in the state machine ", typeid(from).name()));
		}
		std::vector<std::size_t> const& toTransitions = it->second;
		if(std::find(toTransitions.begin(), toTransitions.end(), to.hash_code()) == toTransitions.end()){
			throw std::runtime_error(std::format("Invalid State Transition from {} to {}", typeid(currState).name(), typeid(to).name()));
		}
	}

public:
	explicit StateMachine(State* initialState) : currState{initialState}{};
	~StateMachine(){
		delete currState;
	}

	template<StateLike From, StateLike ...To>
	void enableTransitions(){
		validTransitions[typeid(From).hash_code()] = {typeid(To).hash_code()...};
	}

	void update(){
		State* newState = currState->onLoop();
		
		assertValidTransition(typeid(*currState), typeid(*newState));
		if(newState != currState){
			delete currState;
		}
		currState = newState;
	}
};
