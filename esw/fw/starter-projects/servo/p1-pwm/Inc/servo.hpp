#pragma once

#include "main.h"

class Servo {
public:
	Servo(TIM_HandleTypeDef* tim_handle, uint32_t channel);

	void start_servo();
	void set_servo_angle(int angle);

private:
	TIM_HandleTypeDef* m_tim_handle;
	uint32_t m_channel;
};
