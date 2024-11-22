#pragma once
#include <concepts>
#include <string>

class State {
public:
	virtual ~State() = default;

	virtual auto onLoop() -> State* = 0;
};
