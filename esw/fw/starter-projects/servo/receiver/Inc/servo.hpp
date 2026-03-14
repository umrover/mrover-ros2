#pragma once

#include "main.h"

class Servo {
public:
	Servo(TIM_HandleTypeDef* tim_handle, uint32_t channel, uint32_t max_deg, uint32_t zero_deg_ccr, uint32_t max_deg_ccr);

	auto start_servo() const -> void;
	auto set_angle(double angle) -> void;

private:
	TIM_HandleTypeDef* m_tim_handle;
	uint32_t m_channel;
	uint32_t m_max_deg;
	uint32_t m_zero_deg_ccr;
	uint32_t m_max_deg_ccr;
	uint32_t m_ccr;
};
