#pragma once
#include "state.hpp"

#include <unordered_map>
#include <vector>
#include <typeinfo>
#include <stdexcept>
#include <format>
#include <algorithm>
#include <tuple>

class StateMachine{
private:
	std::string mName;
	State* currState;
	std::unordered_map<std::size_t, std::tuple<std::string, std::vector<std::size_t>, std::vector<std::string>>> validTransitions;

	void assertValidTransition(std::type_info const& from, std::type_info const& to) const {
		auto it = validTransitions.find(from.hash_code());

		if(it == validTransitions.end()){
			throw std::runtime_error(std::format("{} is not in the state machine ", typeid(from).name()));
		}

		std::vector<std::size_t> const& toTransitions = std::get<1>(it->second);
		if(std::find(toTransitions.begin(), toTransitions.end(), to.hash_code()) == toTransitions.end()){
			throw std::runtime_error(std::format("Invalid State Transition from {} to {}", typeid(currState).name(), typeid(to).name()));
		}
	}

public:
	explicit StateMachine(std::string name, State* initialState) : mName{std::move(name)}, currState{initialState}{};
	~StateMachine(){
		delete currState;
	}

	auto getName() const -> std::string const& {
		return mName;
	}

	auto getTransitionTable() const -> std::unordered_map<std::size_t, std::tuple<std::string, std::vector<std::size_t>, std::vector<std::string>>> const&{
		return validTransitions;
	}

	auto getCurrentState() const -> std::string const& {
		return std::get<0>(validTransitions.find(typeid(*currState).hash_code())->second);
	}

	template<StateLike From, StateLike ...To>
	void enableTransitions(){
		validTransitions[typeid(From).hash_code()] = std::make_tuple<std::string, std::vector<std::size_t>, std::vector<std::string>>(From().getName(), {typeid(To).hash_code()...}, {To().getName()...});
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
