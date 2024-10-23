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
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
// extern WWDG_HandleTypeDef hwwdg;

// Measures time since the last quadrature tick reading or the last absolute encoder reading
// Measures time since the last throttle command
// Measures time since the last PIDF update, used for the "D" term
#define VIRTUAL_STOPWATCHES_TIMER &htim7

#define PWM_TIMER_0 &htim1 // H-Bridge PWM
#define PWM_TIMER_CHANNEL_0 TIM_CHANNEL_1

#define PWM_TIMER_1 &htim1
#define PWM_TIMER_CHANNEL_1 TIM_CHANNEL_2

#define PWM_TIMER_2 &htim1
#define PWM_TIMER_CHANNEL_2 TIM_CHANNEL_3

#define QUADRATURE_TICK_TIMER_0 &htim2 // Special encoder timer which externally reads quadrature encoder ticks
#define QUADRATURE_TICK_TIMER_1 &htim3 // Special encoder timer which externally reads quadrature encoder ticks
#define QUADRATURE_TICK_TIMER_2 &htim4 // Special encoder timer which externally reads quadrature encoder ticks

#define GLOBAL_UPDATE_TIMER &htim6 // 20 Hz global timer for: FDCAN send, I2C transaction (absolute encoders)

#define FDCAN_WATCHDOG_TIMER &htim17 // FDCAN watchdog timer that needs to be reset every time a message is received

namespace mrover {

    FDCAN<InBoundMessage> fdcan_bus;
    Controller<NUM_MOTORS> controller;

    auto init() -> void {
        fdcan_bus = FDCAN<InBoundMessage>{DEVICE_ID, DESTINATION_DEVICE_ID, &hfdcan1};
        stopwatches = ControllerStopwatches(VIRTUAL_STOPWATCHES_TIMER);
        controller = Controller{
                PWM_TIMER_1,
                Pin{GPIOB, GPIO_PIN_15},
                Pin{GPIOC, GPIO_PIN_6},
                fdcan_bus,
                FDCAN_WATCHDOG_TIMER,
                QUADRATURE_TICK_TIMER_1,
                QUADRATURE_ELAPSED_TIMER_1,
                THROTTLE_LIMIT_TIMER,
                PIDF_TIMER,
                ABSOLUTE_I2C,
                {
                        LimitSwitch{Pin{LIMIT_0_0_GPIO_Port, LIMIT_0_0_Pin}},
                        LimitSwitch{Pin{LIMIT_0_1_GPIO_Port, LIMIT_0_1_Pin}},
                        // LimitSwitch{Pin{LIMIT_0_2_GPIO_Port, LIMIT_0_2_Pin}},
                        // LimitSwitch{Pin{LIMIT_0_3_GPIO_Port, LIMIT_0_3_Pin}},
                },
        };

        controller.init();

        check(HAL_TIM_Base_Start_IT(GLOBAL_UPDATE_TIMER) == HAL_OK, Error_Handler);
    }

    auto fdcan_received_callback() -> void {
        std::optional<std::pair<FDCAN_RxHeaderTypeDef, InBoundMessage>> received = fdcan_bus.receive();
        if (!received) Error_Handler(); // This function is called WHEN we receive a message so this should never happen

        auto const& [header, message] = received.value();

        auto messageId = std::bit_cast<FDCAN<InBoundMessage>::MessageId>(header.Identifier);

        if (messageId.destination == DEVICE_ID) {
            controller.receive(message);
        }
    }

    auto global_update_callback() -> void {
        controller.update();
        controller.request_absolute_encoder_data();
    }

    auto read_absolute_encoder_data_callback() -> void {
        controller.read_absolute_encoder_data();
    }

    auto update_absolute_encoder_callback() -> void {
        controller.update_absolute_encoder();
    }

    auto update_quadrature_encoder_callback() -> void {
        controller.update_quadrature_encoder();
    }

    auto quadrature_elapsed_timer_expired() -> void {
        controller.quadrature_elapsed_timer_expired();
    }

    auto send_callback() -> void {
        controller.send();
    }

    auto fdcan_watchdog_expired() -> void {
        controller.receive_watchdog_expired();
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
    } else if (htim == FDCAN_WATCHDOG_TIMER) {
        mrover::fdcan_watchdog_expired();
    } else if (htim == VIRTUAL_STOPWATCHES_TIMER) {
        mrover::quadrature_elapsed_timer_expired();
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
    if (htim == QUADRATURE_TICK_TIMER_1) {
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