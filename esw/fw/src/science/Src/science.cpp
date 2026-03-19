#include "main.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_tim.h"
#include "ScienceBoard.hpp"
#include <CANBus1.hpp>
#include <cstddef>
#include <hw/pin.hpp>
#include <logger.hpp>
#include <config.hpp>
#include <string>

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern UART_HandleTypeDef hlpuart1;
extern I2C_HandleTypeDef hi2c3;
extern ADC_HandleTypeDef hadc1;
extern FDCAN_HandleTypeDef hfdcan1;

namespace mrover {
    static constexpr TIM_HandleTypeDef* CO2_TX_TIM = &htim2; // 400ms
    static constexpr TIM_HandleTypeDef* CO2_RX_TIM = &htim3; // 100ms
    static constexpr TIM_HandleTypeDef* CAN_TIM = &htim6; // 500ms
    static constexpr TIM_HandleTypeDef* SENSOR_STATE_TIM = &htim15; // 100ms
    static constexpr TIM_HandleTypeDef* SENSOR_REINIT_TIM = &htim16; // 1000ms
    static constexpr UART_HandleTypeDef* HLPUART = &hlpuart1;
    static constexpr I2C_HandleTypeDef* HI2C = &hi2c3;
    static constexpr FDCAN_HandleTypeDef* HFDCAN = &hfdcan1;
    static constexpr ADC_HandleTypeDef* HADC = &hadc1;
    static constexpr size_t NUM_ADC_CHANNELS = 1;

    UART lpuart;
    ADC<NUM_ADC_CHANNELS> adc;
    FDCAN fdcan;
    ScienceBoard science_board;

    bool adc_free = true;
    bool reinit_sensors = false;
    bool initialized = false;
    std::queue<Sensor> i2c_queue;

    void event_loop() {
        while (true) {
            // check if reinit sensors timer has expired
            if (HAL_I2C_GetState(HI2C) == HAL_I2C_STATE_READY && reinit_sensors) {
                mrover::science_board.reinit_sensors(mrover::i2c_queue);
                reinit_sensors = false;
            }

            // check if there is an i2c message in the queue and the bus is free
            if (HAL_I2C_GetState(HI2C) == HAL_I2C_STATE_READY && !i2c_queue.empty()) 
                science_board.poll_sensor(i2c_queue.front());

            // check if the adc is free to be sampled again
            if (adc_free) {
                adc_free = false;
                science_board.poll_sensor(Sensor::sensor_uv);
            }
        }
    }

    void init() {
        // initialize interfaces
        lpuart = UART{HLPUART, get_uart_options()};
        adc = ADC<NUM_ADC_CHANNELS>{HADC, get_adc_options()};
        fdcan = FDCAN{HFDCAN, get_can_options()};

        // initialize logger
        Logger::init(&lpuart);
        auto& logger = Logger::instance();

        // initialize all sensors
        auto thp_sensor = THP{HI2C};
        auto co2_sensor = CO2Sensor{HI2C};
        auto ozone_sensor = OzoneSensor{HI2C};
        auto oxygen_sensor = OxygenSensor{HI2C};
        auto uv_sensor = UVSensor{&adc, ADC_CHANNEL_0};

        // initialize CAN handler and LEDs
        auto can_handler = CANBus1Handler{&fdcan};
        auto can_tx = Pin{CAN_TX_LED_GPIO_Port, CAN_TX_LED_Pin};
        auto can_rx = Pin{CAN_RX_LED_GPIO_Port, CAN_RX_LED_Pin};

        // initialize debug LEDs
        auto dbg_led1 = Pin{Debug_LED1_GPIO_Port, Debug_LED1_Pin};
        auto dbg_led2 = Pin{Debug_LED2_GPIO_Port, Debug_LED2_Pin};
        auto dbg_led3 = Pin{Debug_LED3_GPIO_Port, Debug_LED3_Pin};

        // create science board object
        logger.info("Initializing science board...");

        HAL_Delay(500); // Delay for sensor startup
        
        science_board = ScienceBoard{thp_sensor, 
                                        co2_sensor, 
                                        ozone_sensor,
                                        oxygen_sensor,
                                        uv_sensor,
                                        can_tx,
                                        can_rx,
                                        dbg_led1,
                                        dbg_led2,
                                        dbg_led3,
                                        can_handler};

        logger.info("Polling sensors...");

        // begin polling sensors
        if (science_board.check_sensor(sensor_thp))
            i2c_queue.push(sensor_thp);

        if (science_board.check_sensor(sensor_oxygen))
            i2c_queue.push(sensor_oxygen);

        if (science_board.check_sensor(sensor_ozone))
            i2c_queue.push(sensor_ozone);

        if (science_board.check_sensor(sensor_co2_tx))
            i2c_queue.push(sensor_co2_tx);

        if (science_board.check_sensor(sensor_uv))
            uv_sensor.sample_sensor();

        logger.info("Starting timers...");

        HAL_TIM_Base_Start_IT(CAN_TIM);
        HAL_TIM_Base_Start_IT(SENSOR_REINIT_TIM);
        HAL_TIM_Base_Start_IT(SENSOR_STATE_TIM);
        
        initialized = true;

        logger.info("Enterring event loop...");
        event_loop();
    }

    void update_i2c_queue() {
        // update sensor value
        Sensor current_sensor = i2c_queue.front();
        i2c_queue.pop();
        science_board.update_sensor(current_sensor);

        if (current_sensor != sensor_co2_tx && current_sensor != sensor_co2_rx)
            i2c_queue.push(current_sensor);
    }

    void handle_i2c_error() {
        // flag bad sensor
        Sensor current_sensor = i2c_queue.front();
        i2c_queue.pop();
        science_board.flag_sensor(current_sensor);
    }
}

extern "C" {
    void PostInit() {
        mrover::init();
    }

    void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim) {
        if (htim == mrover::CO2_TX_TIM) {
            // stop tx timer and request co2 data
            HAL_TIM_Base_Stop_IT(mrover::CO2_TX_TIM);
            mrover::i2c_queue.push(mrover::sensor_co2_tx);
        } else if (htim == mrover::CO2_RX_TIM) {
            // handle co2 sensor
            HAL_TIM_Base_Stop_IT(mrover::CO2_RX_TIM);
            mrover::i2c_queue.push(mrover::sensor_co2_rx);
        } else if (htim == mrover::CAN_TIM) {
            // broadcast CAN data
            if (mrover::initialized)
                mrover::science_board.send_sensor_data();
        } else if (htim == mrover::SENSOR_REINIT_TIM) {
            // attempt to reinitialize any faulty sensors
            if (mrover::initialized)
                mrover::reinit_sensors = true;
        } else if (htim == mrover::SENSOR_STATE_TIM) {
            // broadcast sensor state over CAN
            if (mrover::initialized)
                mrover::science_board.send_sensor_state();
        }
    }

    void HAL_I2C_MasterTxCpltCallback (I2C_HandleTypeDef* hi2c) {
        // reset and start co2 rx timer
        mrover::update_i2c_queue();
        __HAL_TIM_SET_COUNTER(mrover::CO2_RX_TIM, 0);
        HAL_TIM_Base_Start_IT(mrover::CO2_RX_TIM);
    }

    void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef* hi2c) {
        // update co2 and start tx timer
        mrover::update_i2c_queue();
        __HAL_TIM_SET_COUNTER(mrover::CO2_TX_TIM, 0);
        HAL_TIM_Base_Start_IT(mrover::CO2_TX_TIM);
    }

    void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
        // based on the last sensor to make an i2c read request handle response
        mrover::update_i2c_queue();
	}

    void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
        if (!mrover::initialized)
            return;

        while (mrover::fdcan.messages_to_process() > 0) {
            mrover::science_board.handle_request(mrover::i2c_queue);
        }
    }

    void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
        mrover::handle_i2c_error(); 
    }

    void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
        // update uv_index if the UVSensor has been initialized
        if (mrover::initialized)
            mrover::science_board.update_sensor(mrover::sensor_uv);
        
        mrover::adc_free = true;
    }
}