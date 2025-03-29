#pragma once

/**
 * \brief             Virtual state class to describe how states in the state machine should act
 * \see               state_machine.hpp for reference on how the states will be used by the state machine
 */
class State {
public:
	/**
	 * \brief             Virtual destructor for the state class
	 */
	virtual ~State() = default;
	/**
	 * \brief             The function which will be called every loop in the state machine
	 */
	virtual auto onLoop() -> State* = 0;
};
