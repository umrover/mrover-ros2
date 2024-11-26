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

	/**
	 * \brief           Adds the demangled name to the map in the corresponding type hash slot
	 * \param hash		The type hash for the runtime type
	 * \param name		The demangled name of the runtime type
	 */
	void addNameToDecoder(TypeHash hash, std::string const& name){
		constexpr static std::string prefix{"mrover::"};
		TypeHash index = name.find(prefix);
		std::string _name{name};
		_name.replace(index, index + prefix.size(), "");
		decoder[hash] = _name;
	}
public:
	/**
	 * \brief           	Constructor for the StateMachine Class
	 * \param name			The name of the state machine useful for visualization
	 * \param initialState	The initial state which the state machine will begin execution in
	 */
	explicit StateMachine(std::string name, State* initialState) : mName{std::move(name)}, currState{initialState}{};

	~StateMachine(){
		delete currState;
	}

	/**
	 * \brief           Makes a state which the state machine will use for execution.
	 * 					DO NOT CALL THIS FUNCTION AND NOT PASS THE STATE TO THE STATE MACHINE
	 * \param args		The arguments that will be passed to the constructor of the state
	 */
	template<typename T, typename ...Args>
		static auto make_state(Args... args) -> T*{
			static_assert(std::derived_from<T, State>, "State Must Be Derived From The State Class");
			return new T(args...);
		}

	/**
	 * \brief           Returns the name of the state machine
	 * \return          A constant reference to the name of the state machine
	 */
	auto getName() const -> std::string const& {
		return mName;
	}

	/**
	 * \brief           Returns the demangled name of the state at runtime
	 * \param state		A pointer to a state derived object which will have its runtime type analyzed
	 * \return          A constant reference to the demangled state name at runtime
	 */
	auto getStateName(State const* state) const -> std::string const&{
		return decoder.find(typeid(*state).hash_code())->second;
	}

	/**
	 * \brief           Returns the demangled name of the current state in the state machine
	 * \return          A constant reference to the current state's demangled state name
	 */
	auto getCurrentState() const -> std::string const& {
		return getStateName(currState);
	}

	/**
	 * \brief           Returns a map of type hashes to a vector of each type hash
	 * \return          A constant reference to the map describing all valid state transitions
	 */
	auto getTransitionTable() const -> std::unordered_map<TypeHash, std::vector<TypeHash>> const&{
		return mValidTransitions;
	}

	/**
	 * \brief           Takes in a type hash and returns the demangled state name
	 * \return          A constant reference to the demangled state name
	 */
	auto decodeTypeHash(TypeHash hash) const -> std::string const&{
		return decoder.find(hash)->second;
	}

	/**
	 * \brief           Enables the state transition from the first templated type to the subsequent templated types
	 */
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

	/**
	 * \brief        Runs the onLoop function for the state and then transitions to the state returned from that function   
	 */
	void update(){
		State* newState = currState->onLoop();

		assertValidTransition(typeid(*currState), typeid(*newState));
		if(newState != currState){
			delete currState;
		}
		currState = newState;
	}
};
