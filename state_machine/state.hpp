#pragma once
#include <concepts>
#include <string>

template<typename T>
concept StateLike = requires(T state){
	{ state.onLoop() };
	{ state.getName() };
	std::is_default_constructible_v<T>;
};

class State {
public:
	virtual ~State() = default;

	virtual auto onLoop() -> State* = 0;

	[[nodiscard]] virtual auto getName() const -> std::string = 0;
};
