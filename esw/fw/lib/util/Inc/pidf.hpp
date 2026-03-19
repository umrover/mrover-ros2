#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

namespace mrover {

    struct PIDF {
    private:
        // gains
        float m_kp{0.0};
        float m_ki{0.0};
        float m_kd{0.0};
        float m_kff{0.0};

        // settings
        float m_dead_band{0.0};
        float m_out_min{0.0};
        float m_out_max{0.0};

        // input bounds
        std::optional<std::pair<float, float>> m_input_bound;

        // internal state
        float m_total_error{0.0};
        float m_last_error{0.0};

    public:
        /**
         * Calculates the next output signal.
         *
         * @param input     Current sensor reading
         * @param target    Desired setpoint
         * @param dt        Time delta since last call (seconds)
         * @return          Clamped output value
         */
        auto calculate(float const input, float const target, float const dt) -> float {
            if (dt <= 0.0) return m_out_min;

            float error = target - input;

            // continuous input wrapping
            if (m_input_bound) {
                auto [in_min, in_max] = m_input_bound.value();
                if (float const range = in_max - in_min; std::abs(error) > range / 2.0) {
                    if (error > 0) {
                        error -= range;
                    } else {
                        error += range;
                    }
                }
            }

            // anti-windup
            if (float const p_term = m_kp * error; p_term > m_out_min && p_term < m_out_max) {
                m_total_error += error * dt;
            } else {
                m_total_error = 0.0;
            }

            // deadband
            float const error_for_p = (std::abs(error) < m_dead_band) ? 0.0 : error;

            // term calcs
            float const d_term = m_kd * (error - m_last_error) / dt;
            float const i_term = m_ki * m_total_error;
            float const ff_term = m_kff * target;

            // calc result
            float const result = (m_kp * error_for_p) + i_term + d_term + ff_term;

            m_last_error = error;

            return std::clamp(result, m_out_min, m_out_max);
        }

        auto with_p(float const p) -> PIDF& {
            m_kp = p;
            return *this;
        }

        auto with_i(float const i) -> PIDF& {
            m_ki = i;
            return *this;
        }

        auto with_d(float const d) -> PIDF& {
            m_kd = d;
            return *this;
        }

        auto with_ff(float const ff) -> PIDF& {
            m_kff = ff;
            return *this;
        }

        auto with_dead_band(float const dead_band) -> PIDF& {
            m_dead_band = dead_band;
            return *this;
        }

        auto with_input_bound(float min, float max) -> PIDF& {
            m_input_bound = {min, max};
            return *this;
        }

        auto with_output_bound(float const min, float const max) -> PIDF& {
            m_out_min = min;
            m_out_max = max;
            return *this;
        }
    };

} // namespace mrover
