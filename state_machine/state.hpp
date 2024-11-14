#pragma once
#include <concepts>

template<typename T>
concept StateLike = requires(T state){
	{ state.onLoop() };
};

class State {
public:
	virtual ~State() = default;
	virtual auto onLoop() -> State* = 0;
};
