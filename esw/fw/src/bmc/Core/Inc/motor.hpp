#pragma once

#include <CANBus1.hpp>
#include <cinttypes>
#include <err.hpp>
#include <hw/ad8418a.hpp>
#include <hw/hbridge.hpp>
#include <hw/limit_switch.hpp>
// #include <logger.hpp>
#include <pidf.hpp>
#include <variant>

#include "config.hpp"
#include "hw/quadrature.hpp"


namespace mrover {

    class Motor {
        typedef void (*tx_exec_t)(CANBus1Msg_t const& msg);
        typedef void (*can_reset_t)();
        // using tx_exec_t = std::function<void(CANBus1Msg_t const& msg)>;
        // using can_reset_t = std::function<void()>;

        std::optional<HBridge> m_hbridge;
        std::optional<AD8418A> m_current_sensor;
        std::optional<LimitSwitch> m_limit_a;
        std::optional<LimitSwitch> m_limit_b;
        std::optional<QuadratureEncoder> m_quad_encoder;

        tx_exec_t m_message_tx_f{};
        can_reset_t m_initialize_fdcan{};

        std::optional<PIDF> m_pidf{std::nullopt};
        ITimerChannel* m_pidf_elapsed_timer{};

        std::optional<float> m_calibrated_offset{std::nullopt};     // radians
        std::optional<float> m_uncalibrated_position{std::nullopt}; // radians
        std::optional<float> m_velocity{std::nullopt};              // radians/second
        bmc_config_t* m_config_ptr{};
        encoder_mode_t m_encoder_mode{encoder_mode_t::NONE};
        mode_t m_mode{mode_t::STOPPED};
        bmc_error_t m_error{bmc_error_t::NONE};
        float m_target{};
        float m_position{};
        float m_scalar{};

        bool m_enabled{false};
        bool m_limit_a_hit{false};
        bool m_limit_b_hit{false};
        bool m_limit_forward_hit{false};
        bool m_limit_backward_hit{false};

        auto reset() -> void {
            m_mode = mode_t::STOPPED;
            m_error = bmc_error_t::NONE;
            m_target = 0.0f;
        }

        auto sample_encoder() -> void {
            switch (m_encoder_mode) {
                case encoder_mode_t::NONE:
                    m_uncalibrated_position.reset();
                    m_velocity.reset();
                    break;
                case encoder_mode_t::QUAD:
                    m_quad_encoder->update();
                    if (std::optional<EncoderReading> reading = m_quad_encoder->read()) {
                        auto const& [position, velocity] = reading.value();
                        m_uncalibrated_position = position;
                        m_velocity = velocity;
                    } else {
                        m_uncalibrated_position.reset();
                        m_velocity.reset();
                    }
                    break;
                case encoder_mode_t::ABS_SPI:
                    // TODO(eric) impl
                    m_uncalibrated_position.reset();
                    m_velocity.reset();
                    break;
                case encoder_mode_t::ABS_I2C:
                    // TODO(eric) impl
                    m_uncalibrated_position.reset();
                    m_velocity.reset();
                    break;
            }
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
                            auto const target_vel = m_target; // unit of scalar
                            auto const input_vel = m_velocity.value() * m_scalar; // revolutions/sec * scalar
                            auto setpoint_thr = m_pidf->calculate(input_vel, target_vel, m_pidf_elapsed_timer->get_dt());
                            if (setpoint_thr > 0.0f && m_limit_forward_hit) setpoint_thr = 0.0f;
                            if (setpoint_thr < 0.0f && m_limit_backward_hit) setpoint_thr = 0.0f;
                            m_hbridge->write(setpoint_thr);
                        }
                        break;
                    case mode_t::POSITION:
                        if (!m_hbridge->is_on()) m_hbridge->start();
                        {
                            auto const target_pos = m_target; // unit of scalar
                            auto const input_pos = (m_uncalibrated_position.value() - m_calibrated_offset.value()) * m_scalar; // revolutions * scalar
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
            // Logger::instance().info("BMC Initialized with CAN ID 0x%02" PRIX32, m_config_ptr->get<bmc_config_t::can_id>());

            // configure can peripheral
            // m_initialize_fdcan();

            // configure motor parameters
            m_enabled = m_config_ptr->get<bmc_config_t::motor_en>();
            m_hbridge->set_inverted(m_config_ptr->get<bmc_config_t::motor_inv>());
            m_hbridge->set_max_pwm(m_config_ptr->get<bmc_config_t::max_pwm>());

            // configure current sensor
            m_current_sensor->init(get_current_sensor_options());

            // read pidf gains
            m_pidf = PIDF{};
            m_pidf->with_p(m_config_ptr->get<bmc_config_t::k_p>());
            m_pidf->with_i(m_config_ptr->get<bmc_config_t::k_i>());
            m_pidf->with_d(m_config_ptr->get<bmc_config_t::k_d>());
            m_pidf->with_ff(m_config_ptr->get<bmc_config_t::k_f>());
            m_pidf->with_output_bound(-1.0, 1.0);

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
            bool const abs_spi = m_config_ptr->get<bmc_config_t::abs_spi_en>();
            bool const abs_i2c = m_config_ptr->get<bmc_config_t::abs_i2c_en>();
            m_scalar = m_config_ptr->get<bmc_config_t::scalar>();
            if ((quad + abs_spi + abs_i2c) > 1) {
                m_mode = mode_t::FAULT;
                m_error = bmc_error_t::INVALID_FLASH_CONFIG;
            }

            if (quad) {
                m_encoder_mode = encoder_mode_t::QUAD;
                float const phase = m_config_ptr->get<bmc_config_t::quad_phase>() ? 1.0f : -1.0f;
                float const gear_ratio = m_config_ptr->get<bmc_config_t::gear_ratio>();
                float const cpr = m_config_ptr->get<bmc_config_t::quad_cpr>();
                m_quad_encoder->init(phase / gear_ratio, cpr);
            } else if (abs_spi) {
                m_encoder_mode = encoder_mode_t::NONE;
                // TODO(eric) impl
            } else if (abs_i2c) {
                m_encoder_mode = encoder_mode_t::NONE;
                // TODO(eric) impl
            } else {
                m_encoder_mode = encoder_mode_t::NONE;
            }

            __enable_irq();
        }

        template<typename T>
        auto handle(T const& _) const -> void {
        }

        auto handle(BMCProbe const& msg) const -> void {
            // acknowledge probe
            m_message_tx_f(BMCAck{msg.data});
        }

        auto handle(BMCModeCmd const& msg) -> void {
            // stop if not enabled, consume mode only if enabled
            if (!msg.enable)
                m_mode = mode_t::STOPPED;
            else {
                m_mode = static_cast<mode_t>(msg.mode);
                if (m_mode == mode_t::POSITION || m_mode == mode_t::VELOCITY) {
                    if (std::isnan(m_position)) {
                        m_mode = mode_t::FAULT;
                        m_error = bmc_error_t::INVALID_CONFIGURATION_FOR_MODE;
                    }
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

        auto handle(BMCConfigCmd const& msg) -> void {
            // input can either be a request to set a value (apply is set) or read a value (apply not set)
            if (msg.apply) {
                if (m_config_ptr->set_raw(msg.address, msg.value)) {
                    // re-initialize after configuration is modified
                    init();
                }
            } else {
                // send data back as an acknowledgement of the request
                if (uint32_t val{}; m_config_ptr->get_raw(msg.address, val)) {
                    m_message_tx_f(BMCAck{val});
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
                can_reset_t const& initialize_fdcan,
                ITimerChannel* elapsed_timer,
                bmc_config_t* config) : m_message_tx_f{message_tx_f},
                                        m_initialize_fdcan{initialize_fdcan},
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

        auto receive(CANBus1Msg_t const& v) -> void {
            std::visit([this](auto&& value) -> auto {
                handle(value);
            },
                       v);
        }

        auto send_state() -> void {
            // m_current_sensor.update_sensor();

            m_position = [this] -> float {
                if (m_uncalibrated_position && m_calibrated_offset) return (m_uncalibrated_position.value() - m_calibrated_offset.value()) * m_scalar;
                return std::numeric_limits<float>::quiet_NaN();
            }();


            auto const velocity = m_velocity.value_or(std::numeric_limits<float>::quiet_NaN());

            m_message_tx_f(BMCMotorState{
                    static_cast<uint8_t>(m_mode),  // mode
                    static_cast<uint8_t>(m_error), // fault-code
                    m_position,                    // position
                    velocity,                      // velocity
                    m_current_sensor->current(),   // current
                    m_limit_a_hit,                 // limit_a_set
                    m_limit_b_hit,                 // limit_b_set
                    0                              // is_stalled
            });
        }

        auto drive_output() -> void {
            // update limit switch state
            apply_limit(m_limit_a, m_limit_a_hit);
            apply_limit(m_limit_b, m_limit_b_hit);
            m_limit_forward_hit = m_limit_a->is_forward_limit() ? m_limit_a_hit : (m_limit_b->is_forward_limit() ? m_limit_b_hit : false);
            m_limit_backward_hit = !m_limit_a->is_forward_limit() ? m_limit_a_hit : (!m_limit_b->is_forward_limit() ? m_limit_b_hit : false);
            sample_encoder();
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
