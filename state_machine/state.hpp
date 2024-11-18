#pragma once
#include <concepts>
#include <string>

template<typename T>
concept StateLike = requires(T state){
	{ state.onLoop() };
	{ T::getName() };
};

class State {
public:
	virtual ~State() = default;

	virtual auto onLoop() -> State* = 0;
};
