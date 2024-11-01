#include "controller.hpp"

#include <cstdint>

#include <hardware.hpp>
#include <messaging.hpp>

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
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
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
#define QUADRATURE_TICK_TIMER_0 &htim2 // Motor 0
#define QUADRATURE_TICK_TIMER_1 &htim3 // Motor 1
#define QUADRATURE_TICK_TIMER_2 &htim4 // Motor 2

// 20 Hz global timer for: FDCAN send, I2C transaction (absolute encoders)
#define GLOBAL_UPDATE_TIMER &htim6

// Measures time since the last quadrature tick reading or the last absolute encoder reading
// Measures time since the last throttle command
// Measures time since the last PIDF update, used for the "D" term
#define VIRTUAL_STOPWATCHES_TIMER &htim7

// FDCAN watchdog timer that needs to be reset every time a message is received
#define RECEIVE_WATCHDOG_TIMER_0 &htim15 // Motor 0
#define RECEIVE_WATCHDOG_TIMER_1 &htim16 // Motor 1
#define RECEIVE_WATCHDOG_TIMER_2 &htim17 // Motor 2

namespace mrover {

    FDCAN<InBoundMessage> fdcan_bus;
    Controller<NUM_MOTORS> controller;

    auto create_motor_config(std::size_t index) -> MotorConfig {
        switch (index) {
            case 0:
                return {
                        .hbridge_output = PWM_TIMER_0,
                        .hbridge_output_channel = PWM_TIMER_CHANNEL_0,
                        .hbridge_direction = Pin{MOTOR_DIR_0_GPIO_Port, MOTOR_DIR_0_Pin},
                        .receive_watchdog_timer = RECEIVE_WATCHDOG_TIMER_0,
                        .limit_switches = {LimitSwitch{Pin{LIMIT_0_A_GPIO_Port, LIMIT_0_A_Pin}}, LimitSwitch{Pin{LIMIT_0_B_GPIO_Port, LIMIT_0_B_Pin}}},
                        .relative_encoder_tick = QUADRATURE_TICK_TIMER_0,
                        .absolute_encoder_a2_a1 = A2_A1_0};
            case 1:
                return {
                        .hbridge_output = PWM_TIMER_1,
                        .hbridge_output_channel = PWM_TIMER_CHANNEL_1,
                        .hbridge_direction = Pin{MOTOR_DIR_1_GPIO_Port, MOTOR_DIR_1_Pin},
                        .receive_watchdog_timer = RECEIVE_WATCHDOG_TIMER_1,
                        .limit_switches = {LimitSwitch{Pin{LIMIT_1_A_GPIO_Port, LIMIT_1_A_Pin}}, LimitSwitch{Pin{LIMIT_1_B_GPIO_Port, LIMIT_1_B_Pin}}},
                        .relative_encoder_tick = QUADRATURE_TICK_TIMER_1,
                        .absolute_encoder_a2_a1 = A2_A1_1};
            case 2:
                return {
                        .hbridge_output = PWM_TIMER_2,
                        .hbridge_output_channel = PWM_TIMER_CHANNEL_2,
                        .hbridge_direction = Pin{MOTOR_DIR_2_GPIO_Port, MOTOR_DIR_2_Pin},
                        .receive_watchdog_timer = RECEIVE_WATCHDOG_TIMER_2,
                        .limit_switches = {LimitSwitch{Pin{LIMIT_2_A_GPIO_Port, LIMIT_2_A_Pin}}, LimitSwitch{Pin{LIMIT_2_B_GPIO_Port, LIMIT_2_B_Pin}}},
                        .relative_encoder_tick = QUADRATURE_TICK_TIMER_2,
                        .absolute_encoder_a2_a1 = A2_A1_1};
            default:
                return {};
        }
    }

    template<std::size_t... Indices>
    auto create_motor_configs(std::index_sequence<Indices...>) -> std::array<MotorConfig, sizeof...(Indices)> {
        return {create_motor_config(Indices)...};
    }

    auto motor_configs = create_motor_configs(std::make_index_sequence<NUM_MOTORS>{});

    auto init() -> void {
        // fdcan_bus = FDCAN<InBoundMessage>{DEVICE_ID, DESTINATION_DEVICE_ID, &hfdcan1};
        controller = Controller<NUM_MOTORS>{
                fdcan_bus,
                VIRTUAL_STOPWATCHES_TIMER,
                ABSOLUTE_I2C,
                motor_configs};

        controller.init();

        check(HAL_TIM_Base_Start_IT(GLOBAL_UPDATE_TIMER) == HAL_OK, Error_Handler);
    }

    auto fdcan_received_callback() -> void {
        std::optional<std::pair<FDCAN_RxHeaderTypeDef, InBoundMessage>> received = fdcan_bus.receive();
        if (!received) Error_Handler(); // This function is called WHEN we receive a message so this should never happen

        auto const& [header, message] = received.value();

        auto id = std::bit_cast<FDCAN<InBoundMessage>::MessageId>(header.Identifier);

        controller.receive(id, message);
    }

    auto global_update_callback() -> void {
        controller.send();
        controller.request_absolute_encoder_data();
    }

    auto read_absolute_encoder_data_callback() -> void {
        controller.read_absolute_encoder_data();
    }

    auto update_absolute_encoder_callback() -> void {
        controller.update_absolute_encoder();
    }

    template<std::uint8_t MotorIndex>
    auto update_quadrature_encoder_callback() -> void {
        controller.update_quadrature_encoder<MotorIndex>();
    }

    auto send_callback() -> void {
        controller.send();
    }

    template<std::uint8_t MotorIndex>
    auto receive_watchdog_timer_expired() -> void {
        controller.receive_watchdog_expired<MotorIndex>();
    }

    auto virtual_stopwatch_elapsed_callback() -> void {
        controller.virtual_stopwatch_elapsed();
    }
    // void calc_velocity() {
    //     controller.calc_quadrature_velocity();
    // }

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
    } else if (htim == VIRTUAL_STOPWATCHES_TIMER) {
        mrover::virtual_stopwatch_elapsed_callback();
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
    if (htim == QUADRATURE_TICK_TIMER_0) {
        mrover::update_quadrature_encoder_callback<0>();
    } else if (htim == QUADRATURE_TICK_TIMER_1) {
        mrover::update_quadrature_encoder_callback<1>();
    } else if (htim == QUADRATURE_TICK_TIMER_2) {
        mrover::update_quadrature_encoder_callback<2>();
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

// TODO(quintin): Do we need these two callbacks?

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef* hi2c) {}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef* hi2c) {}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c) {
    mrover::update_absolute_encoder_callback();
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c) {
    mrover::read_absolute_encoder_data_callback();
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c) {}
}