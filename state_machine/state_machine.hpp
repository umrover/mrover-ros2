#pragma once
#include "state.hpp"

#include <unordered_map>
#include <vector>
#include <typeinfo>
#include <stdexcept>
#include <format>
#include <algorithm>
#include <tuple>
#include <cxxabi.h>
#include <iostream>

	/**
 * \brief             State Machine class that facilitates transitioning between different states which inherit from the State class
 * \see               state.hpp for reference on creating states
 */
class StateMachine{
private:
	std::string mName;
	State* currState;

	using TypeHash = std::size_t;

	std::unordered_map<TypeHash, std::vector<TypeHash>> mValidTransitions;
	std::unordered_map<TypeHash, std::string> decoder;

        /**
     * \brief           Ensures that transitioning from "from" to "to" is a valid transition
     * \param from		The runtime type of the from state
     * \param to		The runtime type of the to state
     */
	void assertValidTransition(std::type_info const& from, std::type_info const& to) const {
		auto it = mValidTransitions.find(from.hash_code());

		if(it == mValidTransitions.end()){
			throw std::runtime_error(std::format("{} is not in the state machine ", typeid(from).name()));
		}

		std::vector<TypeHash> const& toTransitions = it->second;
		if(std::find(toTransitions.begin(), toTransitions.end(), to.hash_code()) == toTransitions.end()){
			throw std::runtime_error(std::format("Invalid State Transition from {} to {}", typeid(currState).name(), typeid(to).name()));
		}
	}

public:
	explicit StateMachine(std::string name, State* initialState) : mName{std::move(name)}, currState{initialState}{};
	~StateMachine(){
		delete currState;
	}
	
	template<typename T, typename ...Args>
	static auto make_state(Args... args) -> T*{
		return new T(args...);
	}

	auto getName() const -> std::string const& {
		return mName;
	}

	auto getStateName(State const* state) const -> std::string const&{
		return decoder.find(typeid(*state).hash_code())->second;
	}

	auto getCurrentState() const -> std::string const& {
		return decoder.find(typeid(*currState).hash_code())->second;
	}

	auto getTransitionTable() const -> std::unordered_map<TypeHash, std::vector<TypeHash>> const&{
		return mValidTransitions;
	}

	auto decodeTypeHash(TypeHash hash) const -> std::string const&{
		return decoder.find(hash)->second;
	}

	void addNameToDecoder(TypeHash hash, std::string const& name){
		constexpr static std::string prefix{"mrover::"};
		TypeHash index = name.find(prefix);
		std::string _name{name};
		_name.replace(index, index + prefix.size(), "");
		decoder[hash] = _name;
	}

	template<typename From, typename ...To>
	void enableTransitions(){
		static_assert(std::derived_from<From, State>, "From State Must Be Derived From The State Class");
		static_assert((std::derived_from<To, State> && ...), "All States Must Be Derived From The State Class");
		// Add From State To Decoder
		int status = 0;
		char* demangledName = abi::__cxa_demangle(typeid(From).name(), nullptr, nullptr, &status);

		if(status){
			throw std::runtime_error("C++ demangle failed!");
		}

		addNameToDecoder(typeid(From).hash_code(), demangledName);
		free(demangledName);

		mValidTransitions[typeid(From).hash_code()] = {typeid(To).hash_code()...};
	 
		std::vector<std::reference_wrapper<std::type_info const>> types{std::ref(typeid(To))...};
		for(auto const& type : types){
			demangledName = abi::__cxa_demangle(type.get().name(), nullptr, nullptr, &status);
			addNameToDecoder(type.get().hash_code(), demangledName);
			free(demangledName);
		}
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
