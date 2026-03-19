#pragma once

#include <MRoverCAN.hpp>
#include <algorithm>
#include <cinttypes>
#include <err.hpp>
#include <hw/ad8418a.hpp>
#include <hw/hbridge.hpp>
#include <hw/limit_switch.hpp>
#include <hw/quadrature.hpp>
#include <pidf.hpp>
#include <variant>

#include "config.hpp"


namespace mrover {

    class Motor {
        typedef void (*tx_exec_t)(MRoverCANMsg_t const& msg);

        std::optional<HBridge> m_hbridge;
        std::optional<AD8418A> m_current_sensor;
        std::optional<LimitSwitch> m_limit_a;
        std::optional<LimitSwitch> m_limit_b;
        std::optional<QuadratureEncoder> m_quad_encoder;

        tx_exec_t m_message_tx_f{};

        std::optional<PIDF> m_pidf{std::nullopt};
        ITimerChannel* m_pidf_elapsed_timer{};

        std::optional<float> m_calibrated_offset{std::nullopt};     // revolutions
        std::optional<float> m_uncalibrated_position{std::nullopt}; // revolutions
        std::optional<float> m_velocity_raw{std::nullopt};          // revolutions/second
        bmc_config_t* m_config_ptr{};
        encoder_mode_t m_encoder_mode{encoder_mode_t::NONE};
        mode_t m_mode{mode_t::STOPPED};
        bmc_error_t m_error{bmc_error_t::NONE};
        float m_target{};
        float m_position{};
        float m_velocity{};
        float m_rotor_output_ratio{};
        float m_min_position{};
        float m_max_position{};
        float m_min_velocity{};
        float m_max_velocity{};
        float m_stall_current{};
        float m_delta_position{};
        bool m_enabled{false};
        bool m_limit_a_hit{false};
        bool m_limit_b_hit{false};
        bool m_limit_forward_hit{false};
        bool m_limit_backward_hit{false};
        bool m_stalled{false};
        bool m_stall_en{false};

        auto reset() -> void {
            m_mode = mode_t::STOPPED;
            m_error = bmc_error_t::NONE;
            m_target = 0.0f;
        }

        auto sample_encoder() -> void {
            switch (m_encoder_mode) {
                case encoder_mode_t::NONE:
                    m_uncalibrated_position.reset();
                    m_velocity_raw.reset();
                    break;
                case encoder_mode_t::QUAD:
                    m_quad_encoder->update();
                    if (std::optional<EncoderReading> reading = m_quad_encoder->read()) {
                        auto const& [position, velocity] = reading.value();
                        m_uncalibrated_position = position;
                        m_velocity_raw = velocity;
                    } else {
                        m_uncalibrated_position.reset();
                        m_velocity_raw.reset();
                    }
                    break;
            }

            m_position = [this] -> float {
                if (m_uncalibrated_position && m_calibrated_offset) return (m_uncalibrated_position.value() - m_calibrated_offset.value()) * m_rotor_output_ratio;
                return std::numeric_limits<float>::quiet_NaN();
            }();

            m_velocity = [this] -> float {
                if (m_velocity_raw) return m_velocity_raw.value() * m_rotor_output_ratio;
                return std::numeric_limits<float>::quiet_NaN();
            }();
            // Logger::instance().info("velocity: %f", m_velocity);
        }

        auto apply_limit(std::optional<LimitSwitch>& limit, bool& at_limit) -> void {
            if (limit->enabled()) {
                limit->update_limit_switch();
                if (limit->limit_forward()) {
                    at_limit = true;
                    if (std::optional<float> const readjustment_position = limit->get_readjustment_position()) {
                        if (m_uncalibrated_position) {
                            m_calibrated_offset = m_uncalibrated_position.value() - readjustment_position.value();
                        }
                    }
                } else if (limit->limit_backward()) {
                    at_limit = true;
                    if (std::optional<float> const readjustment_position = limit->get_readjustment_position()) {
                        if (m_uncalibrated_position) {
                            m_calibrated_offset = m_uncalibrated_position.value() - readjustment_position.value();
                        }
                    }
                } else {
                    at_limit = false;
                }
            }
        }

        auto detect_stall() -> void {
            // Logger::instance().info("m_stall_en: %u", m_stall_en);
            // Logger::instance().info("current: %u", m_current_sensor);
            // Logger::instance().info("quad: %u", m_quad_encoder);
            // Logger::instance().info("current surge: %s", (m_current_sensor->get_delta_current() > m_delta_current) ? "true" : "false");
            // Logger::instance().info("position change: %s", (m_quad_encoder->get_delta_position() < m_delta_position) ? "true" : "false");

            if (m_stall_en && m_current_sensor && m_current_sensor->current() > m_stall_current) {
                if (m_quad_encoder) {
                    m_stalled = m_quad_encoder->get_delta_position() < m_delta_position;
                } else {
                    m_stalled = true;
                }
                // TODO(eric) what can we do here to protect the motor?
            } else {
                m_stalled = false;
            }
        }

        auto write_output_pwm() -> void {
            if (m_enabled) {
                switch (m_mode) {
                    case mode_t::STOPPED:
                    case mode_t::FAULT:
                        if (m_hbridge->is_on()) m_hbridge->stop();
                        break;
                    case mode_t::THROTTLE:
                        if (!m_hbridge->is_on()) m_hbridge->start();
                        {
                            auto setpoint_thr = m_target; // input is throttle
                            if (setpoint_thr > 0.0f && m_limit_forward_hit) setpoint_thr = 0.0f;
                            if (setpoint_thr < 0.0f && m_limit_backward_hit) setpoint_thr = 0.0f;
                            m_hbridge->write(setpoint_thr);
                        }
                        break;
                    case mode_t::VELOCITY:
                        if (!m_hbridge->is_on()) m_hbridge->start();
                        {
                            auto const target_vel = std::clamp(m_target, m_min_velocity, m_max_velocity); // unit of output
                            auto const input_vel = m_velocity_raw.value() * m_rotor_output_ratio;         // revolutions/sec * output scalar
                            auto setpoint_thr = m_pidf->calculate(input_vel, target_vel, m_pidf_elapsed_timer->get_dt());
                            if (setpoint_thr > 0.0f && m_limit_forward_hit) setpoint_thr = 0.0f;
                            if (setpoint_thr < 0.0f && m_limit_backward_hit) setpoint_thr = 0.0f;
                            m_hbridge->write(setpoint_thr);
                        }
                        break;
                    case mode_t::POSITION:
                        if (!m_hbridge->is_on()) m_hbridge->start();
                        {
                            auto const target_pos = std::clamp(m_target, m_min_position, m_max_position);                                  // unit of output
                            auto const input_pos = (m_uncalibrated_position.value() - m_calibrated_offset.value()) * m_rotor_output_ratio; // revolutions * output scalar
                            auto setpoint_thr = m_pidf->calculate(input_pos, target_pos, m_pidf_elapsed_timer->get_dt());
                            if (setpoint_thr > 0.0f && m_limit_forward_hit) setpoint_thr = 0.0f;
                            if (setpoint_thr < 0.0f && m_limit_backward_hit) setpoint_thr = 0.0f;
                            m_hbridge->write(setpoint_thr);
                        }
                        break;
                }
            }
        }

        /**
         * Initializes the motor as the configuration defines.
         *
         * Should be called after configuration is updated.
         */
        auto init() -> void {
            __disable_irq();

            // configure motor parameters
            m_enabled = m_config_ptr->get<bmc_config_t::motor_en>();
            m_hbridge->set_inverted(m_config_ptr->get<bmc_config_t::motor_inv>());
            m_hbridge->set_max_pwm(m_config_ptr->get<bmc_config_t::max_pwm>());

            // configure current sensor
            m_current_sensor->init(get_current_sensor_options());

            // read pidf gains
            m_pidf = PIDF{};
            m_pidf->with_output_bound(-1.0, 1.0);

            // get velocity clamps
            m_min_position = m_config_ptr->get<bmc_config_t::min_pos>();
            m_max_position = m_config_ptr->get<bmc_config_t::max_pos>();
            m_min_velocity = m_config_ptr->get<bmc_config_t::min_vel>();
            m_max_velocity = m_config_ptr->get<bmc_config_t::max_vel>();

            // init limit switches
            bool en = m_config_ptr->get<bmc_config_t::lim_a_en>();
            bool active_high = m_config_ptr->get<bmc_config_t::lim_a_active_high>();
            bool use_readjust = m_config_ptr->get<bmc_config_t::lim_a_use_readjust>();
            bool is_forward = m_config_ptr->get<bmc_config_t::lim_a_is_forward>();
            float position = m_config_ptr->get<bmc_config_t::limit_a_position>();
            m_limit_a->init(en, active_high, use_readjust, is_forward, position);

            en = m_config_ptr->get<bmc_config_t::lim_b_en>();
            active_high = m_config_ptr->get<bmc_config_t::lim_b_active_high>();
            use_readjust = m_config_ptr->get<bmc_config_t::lim_b_use_readjust>();
            is_forward = m_config_ptr->get<bmc_config_t::lim_b_is_forward>();
            position = m_config_ptr->get<bmc_config_t::limit_b_position>();
            m_limit_b->init(en, active_high, use_readjust, is_forward, position);

            // initialize encoders (error if multiple enabled)
            bool const quad = m_config_ptr->get<bmc_config_t::quad_en>();
            m_rotor_output_ratio = m_config_ptr->get<bmc_config_t::rotor_output_ratio>();

            // initialize stall detection values
            m_stall_en = m_config_ptr->get<bmc_config_t::stall_en>();
            m_stall_current = m_config_ptr->get<bmc_config_t::stall_current>();
            m_delta_position = m_config_ptr->get<bmc_config_t::delta_position>();

            if (quad) {
                m_encoder_mode = encoder_mode_t::QUAD;
                float const phase = m_config_ptr->get<bmc_config_t::quad_phase>() ? 1.0f : -1.0f;
                float const gear_ratio = m_config_ptr->get<bmc_config_t::gear_ratio>();
                float const cpr = m_config_ptr->get<bmc_config_t::quad_cpr>();
                m_quad_encoder->init(phase / gear_ratio, cpr);
            } else {
                m_encoder_mode = encoder_mode_t::NONE;
            }

            __enable_irq();
        }

        template<typename T>
        auto handle(T const& _) const -> void {
        }

        auto handle(ESWProbe const& msg) const -> void {
            // acknowledge probe
            m_message_tx_f(ESWAck{msg.data});
        }

        auto handle(BMCModeCmd const& msg) -> void {
            // stop if not enabled, consume mode only if enabled
            if (!msg.enable)
                m_mode = mode_t::STOPPED;
            else {
                m_mode = static_cast<mode_t>(msg.mode);
                if ((m_mode == mode_t::POSITION || m_mode == mode_t::VELOCITY) && m_encoder_mode == encoder_mode_t::NONE) {
                    m_mode = mode_t::FAULT;
                    m_error = bmc_error_t::INVALID_CONFIGURATION_FOR_MODE;
                }
                if (m_mode == mode_t::POSITION) {
                    if (std::isnan(m_position)) {
                        m_mode = mode_t::FAULT;
                        m_error = bmc_error_t::INVALID_CONFIGURATION_FOR_MODE;
                    }
                }
                if (m_mode == mode_t::POSITION) {
                    m_pidf->with_p(m_config_ptr->get<bmc_config_t::pos_k_p>());
                    m_pidf->with_i(m_config_ptr->get<bmc_config_t::pos_k_i>());
                    m_pidf->with_d(m_config_ptr->get<bmc_config_t::pos_k_d>());
                    m_pidf->with_ff(m_config_ptr->get<bmc_config_t::pos_k_f>());
                }
                if (m_mode == mode_t::VELOCITY) {
                    m_pidf->with_p(m_config_ptr->get<bmc_config_t::vel_k_p>());
                    m_pidf->with_i(m_config_ptr->get<bmc_config_t::vel_k_i>());
                    m_pidf->with_d(m_config_ptr->get<bmc_config_t::vel_k_d>());
                    m_pidf->with_ff(m_config_ptr->get<bmc_config_t::vel_k_f>());
                }
            }
            m_pidf_elapsed_timer->forget_reads();
        }

        auto handle(BMCTargetCmd const& msg) -> void {
            if (!msg.target_valid) return;
            switch (m_mode) {
                case mode_t::STOPPED:
                case mode_t::FAULT:
                    m_target = 0.0f;
                    m_mode = mode_t::FAULT;
                    m_error = bmc_error_t::NO_MODE;
                    break;
                case mode_t::THROTTLE:
                case mode_t::POSITION:
                case mode_t::VELOCITY:
                    m_target = msg.target;
                    break;
            }
        }

        auto handle(ESWConfigCmd const& msg) -> void {
            // input can either be a request to set a value (apply is set) or read a value (apply not set)
            if (msg.apply) {
                if (m_config_ptr->set_raw(msg.address, msg.value)) {
                    // re-initialize after configuration is modified
                    init();
                }
            } else {
                // send data back as an acknowledgement of the request
                if (uint32_t val{}; m_config_ptr->get_raw(msg.address, val)) {
                    m_message_tx_f(ESWAck{val});
                }
            }
        }

        auto handle(BMCResetCmd const& msg) -> void {
            reset();
        }

    public:
        Motor() = default;

        explicit Motor(
                HBridge const& motor_driver,
                AD8418A const& current_sensor,
                LimitSwitch const& limit_a,
                LimitSwitch const& limit_b,
                QuadratureEncoder const& quad_encoder,
                tx_exec_t const& message_tx_f,
                ITimerChannel* elapsed_timer,
                bmc_config_t* config) : m_message_tx_f{message_tx_f},
                                        m_pidf_elapsed_timer{elapsed_timer},
                                        m_config_ptr{config} {
            m_hbridge.emplace(motor_driver);
            m_current_sensor.emplace(current_sensor);
            m_limit_a.emplace(limit_a);
            m_limit_b.emplace(limit_b);
            m_quad_encoder.emplace(quad_encoder);
            reset();
            init();
        }

        auto receive(MRoverCANMsg_t const& v) -> void {
            std::visit([this](auto&& value) -> auto {
                handle(value);
            },
                       v);
        }

        auto send_state() -> void {
            auto const current = [this]() -> float {
                if (m_current_sensor) {
                    m_current_sensor->update_sensor();
                    detect_stall();
                    return m_current_sensor->current();
                }
                return std::numeric_limits<float>::quiet_NaN();
            }();

            m_message_tx_f(BMCMotorState{
                    static_cast<uint8_t>(m_mode),  // mode
                    static_cast<uint8_t>(m_error), // fault-code
                    m_position,                    // position
                    m_velocity,                    // velocity
                    current,                       // current
                    m_limit_a_hit,                 // limit_a_set
                    m_limit_b_hit,                 // limit_b_set
                    m_stalled                      // is_stalled
            });
        }

        auto drive_output() -> void {
            // update limit switch state
            apply_limit(m_limit_a, m_limit_a_hit);
            apply_limit(m_limit_b, m_limit_b_hit);
            m_limit_forward_hit = m_limit_a->is_forward_limit() ? m_limit_a_hit : (m_limit_b->is_forward_limit() ? m_limit_b_hit : false);
            m_limit_backward_hit = !m_limit_a->is_forward_limit() ? m_limit_a_hit : (!m_limit_b->is_forward_limit() ? m_limit_b_hit : false);
            if (m_encoder_mode != encoder_mode_t::NONE) sample_encoder();
            write_output_pwm();
        }

        auto tx_watchdog_lapsed() -> void {
            m_mode = mode_t::FAULT;
            m_error = bmc_error_t::WWDG_EXPIRED;
        }

        auto reset_wwdg() -> void {
            if (m_mode == mode_t::FAULT && m_error == bmc_error_t::WWDG_EXPIRED) {
                m_mode = mode_t::STOPPED;
                m_error = bmc_error_t::NONE;
            }
        }
    };
} // namespace mrover
