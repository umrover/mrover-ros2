#include <CANBus1.hpp>
#include <hw/ad8418a.hpp>
#include <hw/hbridge.hpp>
#include <hw/limit_switch.hpp>
#include <hw/pin.hpp>
#include <hw/quadrature.hpp>
#include <logger.hpp>
#include <serial/fdcan.hpp>
#include <timer.hpp>

#include "config.hpp"
#include "main.h"
#include "motor.hpp"
#include "stm32g431xx.h"
#include "stm32g4xx_hal_tim.h"


extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef hlpuart1;
extern FDCAN_HandleTypeDef hfdcan1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;


namespace mrover {

    static constexpr uint8_t NUM_ADC_CHANNELS = 1;
    static constexpr size_t NUM_ELAPSED_TIMER_CHANNELS = 2;
    static constexpr size_t ELAPSED_TIMER_CH_1 = 0;
    static constexpr size_t ELAPSED_TIMER_CH_2 = 1;

    static constexpr UART_HandleTypeDef* LPUART_1 = &hlpuart1;
    static constexpr ADC_HandleTypeDef* ADC_1 = &hadc1;
    static constexpr FDCAN_HandleTypeDef* FDCAN_1 = &hfdcan1;

    static constexpr TIM_HandleTypeDef* MOTOR_PWM_TIM = &htim1;
    static constexpr TIM_HandleTypeDef* ELAPSED_TIM = &htim2;
    static constexpr TIM_HandleTypeDef* ENCODER_TIM = &htim4;
    static constexpr TIM_HandleTypeDef* TX_TIM = &htim6;        // 10 Hz
    static constexpr TIM_HandleTypeDef* CAN_WWDG_TIM = &htim16; // 10 Hz
    static constexpr TIM_HandleTypeDef* CONTROL_TIM = &htim17;  // 25 Hz

    bmc_config_t config;
    bool volatile initialized = false;
    bool volatile tx_pending = false;
    bool volatile control_update = false;

    // Peripherals
    UART lpuart;
    ADC<NUM_ADC_CHANNELS> adc;
    FDCAN fdcan;

    // Timers
    std::optional<Timer> tx_tim;
    std::optional<Timer> can_wwdg_tim;
    std::optional<Timer> control_tim;
    std::optional<ElapsedTimer<NUM_ELAPSED_TIMER_CHANNELS>> elapsed_timer;

    // Hardware Units
    std::optional<Pin> can_tx;
    std::optional<Pin> can_rx;
    std::optional<CANBus1Handler> can_receiver;
    std::optional<Motor> motor;

    /**
     * Send a CAN message defined in CANBus1.dbc on the bus.
     * @param msg CAN message to send
     */
    auto send_can_message(CANBus1Msg_t const& msg) -> void {
        if (!initialized) return;
        static std::optional<uint8_t> can_id = std::nullopt;
        static std::optional<uint8_t> host_can_id = std::nullopt;

        if (!can_id.has_value()) can_id = config.get<bmc_config_t::can_id>();
        if (!host_can_id.has_value()) host_can_id = config.get<bmc_config_t::host_can_id>();

        can_tx->set();
        can_receiver->send(msg, can_id.value(), host_can_id.value());
        can_tx->reset();
    }

    /**
     * Receive and parse a CAN message over the bus.
     * Message should be of a type defined in CANBus1.dbc
     */
    auto receive_can_message() -> void {
        if (!initialized) return;

        motor->reset_wwdg();

        while (fdcan.messages_to_process() > 0) {
            if (auto const recv = can_receiver->receive(); recv) {
                can_rx->set();
                auto const& msg = *recv;
                motor->receive(msg);
                can_wwdg_tim->reset();
                can_rx->reset();
            }
        }
    }

    /**
     * Initialization sequence for BMC.
     */
    auto init() -> void {
        __disable_irq();
        HAL_DBGMCU_EnableDBGSleepMode();

        // initialize peripherals
        lpuart = UART{LPUART_1, get_uart_options()};
        adc = ADC<NUM_ADC_CHANNELS>{ADC_1, get_adc_options()};
        fdcan = FDCAN{FDCAN_1, get_can_options(&config)};

        // initialize logger
        Logger::init(&lpuart);

        // setup timers
        tx_tim.emplace(TX_TIM, true);              // transmit timer (on interrupt)
        can_wwdg_tim.emplace(CAN_WWDG_TIM, true);  // can watchdog timer (on interrupt)
        control_tim.emplace(CONTROL_TIM, true);    // control timer (update driven output, on interrupt)
        elapsed_timer.emplace(ELAPSED_TIM, false); // pid compute timer

        // timer channels
        auto* pid_timer_handle = elapsed_timer->get_handle(ELAPSED_TIMER_CH_1);
        auto* enc_timer_handle = elapsed_timer->get_handle(ELAPSED_TIMER_CH_2);

        // setup debug LEDs
        can_tx.emplace(CAN_TX_LED_GPIO_Port, CAN_TX_LED_Pin);
        can_rx.emplace(CAN_RX_LED_GPIO_Port, CAN_RX_LED_Pin);

        // initialize fdcan
        can_receiver = CANBus1Handler{&fdcan};

        // setup motor instance
        motor.emplace(
                HBridge{MOTOR_PWM_TIM, TIM_CHANNEL_1, Pin{MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin}},
                AD8418A{&adc, ADC_CHANNEL_0},
                LimitSwitch{Pin{LIMIT_A_GPIO_Port, LIMIT_A_Pin}},
                LimitSwitch{Pin{LIMIT_B_GPIO_Port, LIMIT_B_Pin}},
                QuadratureEncoder{ENCODER_TIM, enc_timer_handle},
                send_can_message,
                pid_timer_handle,
                &config);

        // set initialization state and initial error state
        initialized = true;
        __enable_irq();
    }

    [[noreturn]] auto loop() -> void {
        for (;;) {
            // TODO(eric) feels like FreeRTOS would be nice here
            if (tx_pending) {
                motor->send_state();
                tx_pending = false;
            }
            if (control_update) {
                motor->drive_output();
                control_update = false;
            }
            __DSB();
            HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
        }
    }

    /**
     * Callback for timer periods elapsing.
     * Timers have to be started with "HAL_TIM_Base_Start_IT" for this interrupt to work for them.
     * @param htim The timer whose period elapsed
     */
    auto timer_elapsed_callback(TIM_HandleTypeDef const* htim) -> void {
        if (!initialized) return;
        if (htim == TX_TIM) {
            tx_pending = true;
        } else if (htim == CAN_WWDG_TIM) {
            motor->tx_watchdog_lapsed();
        } else if (htim == CONTROL_TIM) {
            control_update = true;
        }
    }

    /**
     * Callback enabling asynchronous serial logs via UART/DMA.
     * @param huart UART handle from callback
     */
    auto uart_tx_callback(UART_HandleTypeDef const* huart) -> void {
        if (huart == LPUART_1) {
            lpuart.handle_tx_complete();
        }
    }

} // namespace mrover

extern "C" {

void PostInit() {
    mrover::init();
}

void Loop() {
    mrover::loop();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    mrover::timer_elapsed_callback(htim);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
    mrover::receive_can_message();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    mrover::uart_tx_callback(huart);
}

// TODO(eric) implement
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {}
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef* hfdcan) {}
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t ErrorStatusITs) {}
// void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c) {}
// void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c) {}
// void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c) {}
}
