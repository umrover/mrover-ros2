#include <cstdint>

#include <common.hpp>
#include <hardware.hpp>
#include <hardware_tim.hpp>
#include <hbridge.hpp>
#include <messaging.hpp>
#include <motor.hpp>

#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1;

extern I2C_HandleTypeDef hi2c1;
#define ABSOLUTE_I2C &hi2c1

/**
 * For each repeating timer, the update rate is determined by the .ioc file.
 *
 * Specifically the ARR value. You can use the following equation: ARR = (MCU Clock Speed) / (Update Rate) / (Prescaler + 1) - 1
 * For the STM32G4 we have a 140 MHz clock speed configured.
 *
 * You must also set auto reload to true so the interrupt gets called on a cycle.
 */

extern TIM_HandleTypeDef htim1;
// extern TIM_HandleTypeDef htim2;
// extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

// H-Bridge PWM
#define PWM_TIMER_0 &htim1 // Motor 0
#define PWM_TIMER_CHANNEL_0 TIM_CHANNEL_1
#define PWM_TIMER_1 &htim1 // Motor 1
#define PWM_TIMER_CHANNEL_1 TIM_CHANNEL_2
#define PWM_TIMER_2 &htim1 // Motor 2
#define PWM_TIMER_CHANNEL_2 TIM_CHANNEL_3

// Special encoder timer which externally reads quadrature encoder ticks
#define QUADRATURE_TICK_TIMER &htim4
// #define QUADRATURE_TICK_TIMER_0 &htim4
// #define QUADRATURE_TICK_TIMER_1 &htim3
// #define QUADRATURE_TICK_TIMER_2 &htim2

// 20 Hz global timer for: FDCAN send, I2C transaction (absolute encoders)
#define GLOBAL_UPDATE_TIMER &htim6

// Measures time since the last quadrature tick reading or the last absolute encoder reading
#define ENCODER_ELAPSED_TIMER &htim7

// Measures time since the last PIDF update, used for the "D" term
#define PIDF_TIMER &htim8

// FDCAN watchdog timer that needs to be reset every time a message is received
#define RECEIVE_WATCHDOG_TIMER_0 &htim15 // Motor 0
#define RECEIVE_WATCHDOG_TIMER_1 &htim16 // Motor 1
#define RECEIVE_WATCHDOG_TIMER_2 &htim17 // Motor 2

namespace mrover {
    typedef typename std::array<Motor, NUM_MOTORS>::iterator MotorIterator;

    FDCAN<InBoundMessage> fdcan_bus;
    std::array<Motor, NUM_MOTORS> motors;
    Pin can_tx_led, can_rx_led;
    MotorIterator motor_with_encoder;

    auto get_motor_with_encoder() -> MotorIterator {
        for (auto motor_it = motors.begin(); motor_it != motors.end(); ++motor_it) {
            if (motor_it->has_quadrature_encoder_configured()) {
                return motor_it;
            }
        }
        return motors.end();
    }

    constexpr auto create_motor(std::size_t index) -> Motor {
        switch (index) {
            case 0:
                return Motor(
                        DEVICE_ID_0,
#ifdef DUAL_DIRECTION
                        HBridge{PWM_TIMER_0, PWM_TIMER_CHANNEL_0, Pin{MOTOR_DIR_0_GPIO_Port, MOTOR_DIR_0_Pin}, Pin{MOTOR_DIR_1_GPIO_Port, MOTOR_DIR_1_Pin}},
#else
                        HBridge{PWM_TIMER_0, PWM_TIMER_CHANNEL_0, Pin{MOTOR_DIR_0_GPIO_Port, MOTOR_DIR_0_Pin}},
#endif
                        RECEIVE_WATCHDOG_TIMER_0,
                        {LimitSwitch{Pin{LIMIT_0_A_GPIO_Port, LIMIT_0_A_Pin}}, LimitSwitch{Pin{LIMIT_0_B_GPIO_Port, LIMIT_0_B_Pin}}},
                        ENCODER_ELAPSED_TIMER,
                        QUADRATURE_TICK_TIMER,
                        ABSOLUTE_I2C,
                        A2_A1_0,
                        PIDF_TIMER);
            case 1:
                return Motor(
                        DEVICE_ID_1,
#ifdef DUAL_DIRECTION
                        HBridge{PWM_TIMER_1, PWM_TIMER_CHANNEL_1, Pin{MOTOR_DIR_2_GPIO_Port, MOTOR_DIR_2_Pin}, Pin{MOTOR_DIR_3_GPIO_Port, MOTOR_DIR_3_Pin}},
#else
                        HBridge{PWM_TIMER_1, PWM_TIMER_CHANNEL_1, Pin{MOTOR_DIR_1_GPIO_Port, MOTOR_DIR_1_Pin}},
#endif
                        RECEIVE_WATCHDOG_TIMER_1,
                        {LimitSwitch{Pin{LIMIT_1_A_GPIO_Port, LIMIT_1_A_Pin}}, LimitSwitch{Pin{LIMIT_1_B_GPIO_Port, LIMIT_1_B_Pin}}},
                        ENCODER_ELAPSED_TIMER,
                        QUADRATURE_TICK_TIMER,
                        ABSOLUTE_I2C,
                        A2_A1_1,
                        PIDF_TIMER);
            case 2:
                return Motor(
                        DEVICE_ID_2,
#ifdef DUAL_DIRECTION
                        HBridge{PWM_TIMER_2, PWM_TIMER_CHANNEL_2, Pin{MOTOR_DIR_4_GPIO_Port, MOTOR_DIR_4_Pin}, Pin{MOTOR_DIR_5_GPIO_Port, MOTOR_DIR_5_Pin}},
#else
                        HBridge{PWM_TIMER_2, PWM_TIMER_CHANNEL_2, Pin{MOTOR_DIR_2_GPIO_Port, MOTOR_DIR_2_Pin}},
#endif
                        RECEIVE_WATCHDOG_TIMER_2,
                        {LimitSwitch{Pin{LIMIT_2_A_GPIO_Port, LIMIT_2_A_Pin}}, LimitSwitch{Pin{LIMIT_2_B_GPIO_Port, LIMIT_2_B_Pin}}},
                        ENCODER_ELAPSED_TIMER,
                        QUADRATURE_TICK_TIMER,
                        ABSOLUTE_I2C,
                        A2_A1_2,
                        PIDF_TIMER);
            default:
                return {};
        }
    }

    template<std::size_t... Indices>
    constexpr auto create_motor_array_impl(std::index_sequence<Indices...>) -> std::array<Motor, sizeof...(Indices)> {
        return {create_motor(Indices)...};
    }

    template<std::size_t N>
    constexpr auto create_motor_array() -> std::array<Motor, N> {
        return create_motor_array_impl(std::make_index_sequence<N>{});
    }

    auto init() -> void {
        fdcan_bus = FDCAN<InBoundMessage>(&hfdcan1);
        motors = create_motor_array<NUM_MOTORS>();
        can_tx_led = Pin{CAN_TX_LED_GPIO_Port, CAN_TX_LED_Pin};
        can_rx_led = Pin{CAN_RX_LED_GPIO_Port, CAN_RX_LED_Pin};

        motor_with_encoder = motors.end();

        fdcan_bus.add_filter(DEVICE_ID_0);
        fdcan_bus.add_filter(DEVICE_ID_1);
        fdcan_bus.add_filter(DEVICE_ID_2);

        fdcan_bus.start();

        check(HAL_TIM_Base_Start_IT(GLOBAL_UPDATE_TIMER) == HAL_OK, Error_Handler);
        check(HAL_TIM_Base_Start(PIDF_TIMER) == HAL_OK, Error_Handler);
    }

    /**
     * \brief Called from the FDCAN interrupt handler when a new message is received, updating \link m_inbound \endlink and processing it.
     */
    auto fdcan_received_callback() -> void {
        std::optional<std::pair<FDCAN_RxHeaderTypeDef, InBoundMessage>> received = fdcan_bus.receive();
        if (!received) Error_Handler(); // This function is called WHEN we receive a message so this should never happen

        auto const& [header, message] = received.value();

        auto const id = std::bit_cast<FDCAN<InBoundMessage>::MessageId>(header.Identifier);

        for (std::size_t i = 0; i < NUM_MOTORS; ++i) {
            if (motors[i].get_id() == id.destination) {
                can_rx_led.write(GPIO_PIN_SET);
                motors[i].receive(message);
                motors[i].update();
                if (motor_with_encoder == motors.end()) {
                    if (motors[i].has_quadrature_encoder_configured() || motors[i].has_absolute_encoder_configured()) {
                        motor_with_encoder = motors.begin() + i;
                    }
                }
                can_rx_led.write(GPIO_PIN_RESET);
                break;
            }
        }
    }

    /**
     * \brief Send out the current outbound status message of each motor.
     *
     * The update rate should be limited to avoid hammering the FDCAN bus.
     */
    auto send_motor_statuses() -> void {
        for (auto const& motor: motors) {
            can_tx_led.write(GPIO_PIN_SET);
            if (bool success = fdcan_bus.broadcast(motor.get_outbound(), motor.get_id(), DESTINATION_DEVICE_ID); !success) {
                fdcan_bus.reset();
            }
            can_tx_led.write(GPIO_PIN_RESET);
        }
    }

    auto encoder_elapsed_expired_callback() -> void {
        if (motor_with_encoder != motors.end()) {
            motor_with_encoder->encoder_elapsed_expired();
        }
    }

    auto request_absolute_encoder_data_callback() -> void {
        if (motor_with_encoder != motors.end()) {
            motor_with_encoder->request_absolute_encoder_data();
        }
    }

    auto global_update_callback() -> void {
        send_motor_statuses();
        request_absolute_encoder_data_callback();
    }

    auto read_absolute_encoder_data_callback() -> void {
        motor_with_encoder->read_absolute_encoder_data();
    }

    auto update_absolute_encoder_callback() -> void {
        motor_with_encoder->update_absolute_encoder();
    }

    auto update_quadrature_encoder_callback() -> void {
        if (motor_with_encoder != motors.end()) {
            motor_with_encoder->update_quadrature_encoder();
        }
    }

    template<std::uint8_t MotorIndex>
    auto receive_watchdog_timer_expired() -> void {
        motors[MotorIndex].receive_watchdog_expired();
    }

} // namespace mrover

// TOOD: is this really necesssary?
extern "C" {

void HAL_PostInit() {
    mrover::init();
}

/**
 * These are interrupt handlers. They are called by the HAL.
 *
 * These are set up in the .ioc file.
 * They have to be enabled in the NVIC settings.
 * It is important th
*/

/**
 * \note Timers have to be started with "HAL_TIM_Base_Start_IT" for this interrupt to work for them.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    // HAL_WWDG_Refresh(&hwwdg);
    if (htim == GLOBAL_UPDATE_TIMER) {
        mrover::global_update_callback();
    } else if (htim == RECEIVE_WATCHDOG_TIMER_0) {
        mrover::receive_watchdog_timer_expired<0>();
    } else if (htim == RECEIVE_WATCHDOG_TIMER_1) {
        mrover::receive_watchdog_timer_expired<1>();
    } else if (htim == RECEIVE_WATCHDOG_TIMER_2) {
        mrover::receive_watchdog_timer_expired<2>();
    } else if (htim == ENCODER_ELAPSED_TIMER) {
        mrover::encoder_elapsed_expired_callback();
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
    if (htim == QUADRATURE_TICK_TIMER) {
        mrover::update_quadrature_encoder_callback();
    }
}

/**
 * \note FDCAN1 has to be configured with "HAL_FDCAN_ActivateNotification" and started with "HAL_FDCAN_Start" for this interrupt to work.
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
        mrover::fdcan_received_callback();
    } else {
        // Mailbox is full OR we lost a frame
        Error_Handler();
    }
}

// TODO(quintin): Error handling

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef* hfdcan) {}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t ErrorStatusITs) {}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c) {
    mrover::update_absolute_encoder_callback();
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c) {
    mrover::read_absolute_encoder_data_callback();
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c) {}
}
