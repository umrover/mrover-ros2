#include "servo.hpp"

Servo::Servo(TIM_HandleTypeDef* tim_handle, uint32_t channel, uint32_t max_deg, uint32_t zero_deg_ccr, uint32_t max_deg_ccr) :
	m_tim_handle(tim_handle), m_channel(channel), m_max_deg(max_deg), m_zero_deg_ccr(zero_deg_ccr), m_max_deg_ccr(max_deg_ccr) {}

auto Servo::start_servo() const -> void {
	HAL_TIM_PWM_Start(m_tim_handle, m_channel);
}

auto Servo::set_angle(double const angle) -> void {
	m_ccr = m_zero_deg_ccr + ((angle / m_max_deg) * (m_max_deg_ccr - m_zero_deg_ccr));
	__HAL_TIM_SET_COMPARE(m_tim_handle, m_channel, m_ccr);
}
