#include "CO2Sensor.hpp"
#include "OxygenSensor.hpp"
#include "OzoneSensor.hpp"
#include "UVSensor.hpp"
#include "main.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_tim.h"
#include "thp_sensor.hpp"
#include <CANBus1.hpp>
#include <hw/pin.hpp>
#include <logger.hpp>
#include <config.hpp>
#include <queue>

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef hlpuart1;
extern I2C_HandleTypeDef hi2c3;
extern ADC_HandleTypeDef hadc1;
extern FDCAN_HandleTypeDef hfdcan1;

namespace mrover {
    enum I2CSensor {
        sensor_co2_rx = 0,
        sensor_co2_tx = 1,
        sensor_thp = 2,
        sensor_ozone = 3,
        sensor_oxygen = 4,
    };

    static constexpr TIM_HandleTypeDef* CO2_TX_TIM = &htim2; // 400ms
    static constexpr TIM_HandleTypeDef* CO2_RX_TIM = &htim3; // 100ms
    static constexpr TIM_HandleTypeDef* CAN_TIM = &htim6; // 500ms
    static constexpr UART_HandleTypeDef* HLPUART = &hlpuart1;
    static constexpr I2C_HandleTypeDef* HI2C = &hi2c3;
    static constexpr FDCAN_HandleTypeDef* HFDCAN = &hfdcan1;
    static constexpr ADC_HandleTypeDef* HADC = &hadc1;
    static constexpr size_t NUM_ADC_CHANNELS = 1;

    UART lpuart;
    ADC<NUM_ADC_CHANNELS> adc;
    FDCAN fdcan;
    THP thp_sensor;
    CO2Sensor co2_sensor;
    THP_data thp_data;
    OzoneSensor ozone_sensor;
    OxygenSensor oxygen_sensor;
    UVSensor uv_sensor;
    Pin can_tx;
    Pin can_rx;
    CANBus1Handler can_handler;

    sb_config_t config;

    float co2 = 0;
    float ozone = 0;
    float oxygen = 0;
    float uv_index = 0;
    bool adc_free = true;
    bool initialized = false;
    std::queue<I2CSensor> i2c_queue;

    void log_data() {
        if (!initialized)
            return;

        static auto& logger = Logger::instance();

        logger.info("uv index: %f", uv_index);
        logger.info("temp: %f", thp_data.temp);
        logger.info("humidity: %f", thp_data.humidity);
        logger.info("pressure: %f", thp_data.pressure);
        logger.info("oxygen: %f", oxygen);
        logger.info("ozone: %f", ozone);
        logger.info("co2: %f", co2);
    }

    void send_can() {
        if (!initialized)
            return;

        size_t size = i2c_queue.size();
        can_tx.set();
        const CANBus1Msg_t msg = SCISensorData(uv_index, thp_data.temp, thp_data.humidity, thp_data.pressure, oxygen, ozone, co2);
        // can_handler.send(msg, config.get<sb_config_t::can_id>(), config.get<sb_config_t::host_can_id>());
        can_handler.send(msg, 0x70, 0x10);
        can_tx.reset();
    }

    void init() {
        lpuart = UART{HLPUART, get_uart_options()};
        Logger::init(&lpuart);
        auto& logger = Logger::instance();

        adc = ADC<NUM_ADC_CHANNELS>{HADC, get_adc_options()};

        // initialize all sensors
        thp_sensor = THP{HI2C};
	    thp_sensor.init();
        co2_sensor = CO2Sensor{HI2C};
        co2_sensor.init();
        ozone_sensor = OzoneSensor(HI2C);
	    ozone_sensor.init();
        oxygen_sensor = OxygenSensor(HI2C);
	    oxygen_sensor.init();
        uv_sensor = UVSensor(&adc, ADC_CHANNEL_0);

        fdcan = FDCAN{HFDCAN, get_can_options()};
        can_handler = CANBus1Handler{&fdcan};
        can_tx = Pin{CAN_TX_LED_GPIO_Port, CAN_TX_LED_Pin};
        can_rx = Pin{CAN_RX_LED_GPIO_Port, CAN_RX_LED_Pin};

        Pin dbg_led1 = Pin{Debug_LED1_GPIO_Port, Debug_LED1_Pin};
        Pin dbg_led2 = Pin{Debug_LED2_GPIO_Port, Debug_LED2_Pin};
        Pin dbg_led3 = Pin{Debug_LED3_GPIO_Port, Debug_LED3_Pin};

        dbg_led1.set();
        dbg_led2.set();
        dbg_led3.set();
        can_tx.set();
        can_rx.set();

        logger.info("Polling sensors...");

        // begin polling sensors
        HAL_TIM_Base_Start_IT(mrover::CO2_TX_TIM);
        HAL_TIM_Base_Start_IT(mrover::CAN_TIM);

        i2c_queue.push(mrover::sensor_thp);
        i2c_queue.push(mrover::sensor_oxygen);
        i2c_queue.push(mrover::sensor_ozone);
        uv_sensor.sample_sensor();

        can_tx.set();
        
        initialized = true;

        while (true) {
            // check if there is an i2c message in the queue and the bus is free
            if (!i2c_queue.empty() && !__HAL_I2C_GET_FLAG(HI2C, I2C_FLAG_BUSY)) {
                I2CSensor current_sensor = i2c_queue.front();
                // handle current sensor based on sensor
                if (current_sensor == sensor_co2_tx)
                    co2_sensor.request_co2();
                else if (current_sensor == sensor_co2_rx)
                    co2_sensor.receive_buf();
                else if (current_sensor == sensor_thp)
                    thp_sensor.read_thp();
                else if (current_sensor == sensor_oxygen)
                    oxygen_sensor.read_oxygen();
                else if (current_sensor == sensor_ozone)
                    ozone_sensor.read_ozone();
            }

            // check if the adc is free to be sampled again
            if (adc_free) {
                adc_free = false;
                mrover::uv_sensor.sample_sensor();
            }
        }
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
            mrover::log_data();
            mrover::send_can();
        }
    }

    void HAL_I2C_MasterTxCpltCallback (I2C_HandleTypeDef* hi2c) {
        // reset and start co2 rx timer
        mrover::i2c_queue.pop();
        __HAL_TIM_SET_COUNTER(mrover::CO2_RX_TIM, 0);
        HAL_TIM_Base_Start_IT(mrover::CO2_RX_TIM);
    }

    void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef* hi2c) {
        // update co2 and reset tx timer
        mrover::i2c_queue.pop();
        mrover::co2 = mrover::co2_sensor.update_co2();
        __HAL_TIM_SET_COUNTER(mrover::CO2_TX_TIM, 0);
        HAL_TIM_Base_Start_IT(mrover::CO2_TX_TIM);
    }

    void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
        // based on the last sensor to make an i2c read request handle response
        if (mrover::i2c_queue.front() == mrover::sensor_thp) {
            mrover::i2c_queue.pop();
		    mrover::thp_data = mrover::thp_sensor.update_thp();
            mrover::i2c_queue.push(mrover::sensor_thp);
        } else if (mrover::i2c_queue.front() == mrover::sensor_oxygen) {
            mrover::i2c_queue.pop();
            mrover::oxygen = mrover::oxygen_sensor.update_oxygen();
            mrover::i2c_queue.push(mrover::sensor_oxygen);
        } else if (mrover::i2c_queue.front() == mrover::sensor_ozone) {
            mrover::i2c_queue.pop();
            mrover::ozone = mrover::ozone_sensor.update_ozone();
            mrover::i2c_queue.push(mrover::sensor_ozone);
        }
	}

    void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
        if (mrover::initialized)
            mrover::uv_index = mrover::uv_sensor.update_uv();
        
        mrover::adc_free = true;
    }
}