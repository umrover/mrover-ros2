#include "motor.hpp"

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
 * For the STM32G4 we have a 170 MHz clock speed configured.
 *
 * You must also set auto reload to true so the interrupt gets called on a cycle.
 */

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
// extern WWDG_HandleTypeDef hwwdg;

#define QUADRATURE_TICK_TIMER &htim4    // Special encoder timer which externally reads quadrature encoder ticks

// Measures time since:
// The last absolute/quad encoder reading
// The last throttle command
// The last PIDF update
#define GENERIC_ELAPSED_TIMER &htim2
constexpr auto GENERIC_ELAPSED_FREQUENCY = mrover::Hertz{1000000};

#define PWM_TIMER &htim15 // H-Bridge PWM
#define PWM_TIMER_CHANNEL TIM_CHANNEL_1

#define GLOBAL_UPDATE_TIMER &htim7   // 20 Hz global timer for: FDCAN send, I2C transaction (absolute encoders), read quad encoder
#define FDCAN_WATCHDOG_TIMER &htim16 // FDCAN watchdog timer that needs to be reset every time a message is received

namespace mrover {

    FDCAN<InBoundMessage> fdcan_bus;
    Motor motor;

    auto init() -> void {
        fdcan_bus = FDCAN<InBoundMessage>{&hfdcan1};
        motor = Motor{
                DEVICE_ID,
                HBridge{PWM_TIMER, PWM_TIMER_CHANNEL, Pin{MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin}},
				FDCAN_WATCHDOG_TIMER,
                {LimitSwitch{Pin{LIMIT_0_GPIO_Port, LIMIT_0_Pin}}, LimitSwitch{Pin{LIMIT_1_GPIO_Port, LIMIT_1_Pin}}},
                GENERIC_ELAPSED_TIMER, GENERIC_ELAPSED_FREQUENCY,
                QUADRATURE_TICK_TIMER,
                ABSOLUTE_I2C,
                A2_A1
        };

        // fdcan_bus.add_filter(DEVICE_ID);
        fdcan_bus.start();

        check(HAL_TIM_Base_Start(GENERIC_ELAPSED_TIMER) == HAL_OK, Error_Handler);
        check(HAL_TIM_Base_Start_IT(GLOBAL_UPDATE_TIMER) == HAL_OK, Error_Handler);
    }

    auto fdcan_received_callback() -> void {
        std::optional<std::pair<FDCAN_RxHeaderTypeDef, InBoundMessage>> received = fdcan_bus.receive();
        if (!received) Error_Handler(); // This function is called WHEN we receive a message so this should never happen

        auto const& [header, message] = received.value();

        auto messageId = std::bit_cast<FDCAN<InBoundMessage>::MessageId>(header.Identifier);

        if (messageId.destination == DEVICE_ID) {
            motor.receive(message);
            motor.update();
        }
    }

    auto send_motor_status() -> void {
		if (bool success = fdcan_bus.broadcast(motor.get_outbound(), motor.get_id(), DESTINATION_DEVICE_ID); !success) {
			fdcan_bus.reset();
		}
    }

	auto update_relative_encoder_callback() -> void {
		motor.update_quadrature_encoder();
	}

	auto request_absolute_encoder_data_callback() -> void {
		motor.request_absolute_encoder_data();
	}

	auto read_absolute_encoder_data_callback() -> void {
		motor.read_absolute_encoder_data();
	}

	auto update_absolute_encoder_callback() -> void {
		motor.update_absolute_encoder();
	}

	auto update_quadrature_encoder_callback() -> void {
		motor.update_quadrature_encoder();
	}


	auto global_update_callback() -> void {
		send_motor_status();
		request_absolute_encoder_data_callback();
		update_relative_encoder_callback();
	}

    auto fdcan_watchdog_expired() -> void {
        motor.receive_watchdog_expired();
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
    } else if (htim == FDCAN_WATCHDOG_TIMER) {
        mrover::fdcan_watchdog_expired();
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
