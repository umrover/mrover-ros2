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

class StateMachine{
private:
	std::string mName;
	State* currState;
	std::unordered_map<std::size_t, std::vector<std::size_t>> mValidTransitions;
	std::unordered_map<std::size_t, std::string> decoder;

	void assertValidTransition(std::type_info const& from, std::type_info const& to) const {
		auto it = mValidTransitions.find(from.hash_code());

		if(it == mValidTransitions.end()){
			throw std::runtime_error(std::format("{} is not in the state machine ", typeid(from).name()));
		}

		std::vector<std::size_t> const& toTransitions = it->second;
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

	auto getStateName(State const* state) const -> std::string const&{
		return decoder.find(typeid(*state).hash_code())->second;
	}

	auto getCurrentState() const -> std::string const& {
		return decoder.find(typeid(*currState).hash_code())->second;
	}

	auto getTransitionTable() const -> std::unordered_map<std::size_t, std::vector<std::size_t>> const&{
		return mValidTransitions;
	}

	auto decodeTypeHash(std::size_t hash) const -> std::string const&{
		return decoder.find(hash)->second;
	}

	void addNameToDecoder(std::size_t hash, std::string const& name){
		std::string _name = name;
		for(char& c : _name){
			if(c == ':'){
				c = '_';
			}
		}
		decoder[hash] = _name;
	}

	template<StateLike From, StateLike ...To>
	void enableTransitions(){
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
			int status = 0;
			char* demangledName = abi::__cxa_demangle(type.get().name(), nullptr, nullptr, &status);
			std::cout << demangledName << std::endl;
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
