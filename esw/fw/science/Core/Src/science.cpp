#include "MethaneSensor.hpp"
#include "OxygenSensor.hpp"
#include "TempHumiditySensor.hpp"
#include "UVSensor.hpp"
#include "hardware.hpp"
#include "hardware_adc.hpp"
#include "messaging_science.hpp"
#include "diag_temp_sensor.hpp"
#include "heater.hpp"
#include <memory>

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

#define JETSON_ADDRESS 0x10
#define SCIENCE_BOARD_ID 0x50

namespace mrover {

    std::shared_ptr<ADCSensor> adc_sensor1 = std::make_shared<ADCSensor>(&hadc1, 6);
    ADCSensor adc_sensor2 = ADCSensor(&hadc2, 0);

    UVSensor uv_sensor;
    OxygenSensor oxygen_sensor;
    TempHumiditySensor th_sensor;
    FDCAN<InBoundScienceMessage> fdcan_bus;
    OutBoundScienceMessage science_out;

    std::array<Heater, 2> m_heaters;
	std::array<Pin, 1> m_white_leds;

    void event_loop() {
        while (true) {}
    }

    void init() {
    	uv_sensor = UVSensor(&adc_sensor2, 0);
		oxygen_sensor = OxygenSensor(&hi2c2);
		th_sensor = TempHumiditySensor(&hi2c3);
		fdcan_bus = FDCAN<InBoundScienceMessage>(&hfdcan1);

        std::array<DiagTempSensor, 2> diag_temp_sensors =
		{
				DiagTempSensor{adc_sensor1, 0},
				DiagTempSensor{adc_sensor1, 1},

		};
        std::array<Pin, 2> heater_pins =
        {
        		Pin{HEATER_1_GPIO_Port, HEATER_1_Pin},
				Pin{HEATER_2_GPIO_Port, HEATER_2_Pin},
        };
        std::array<Pin, 1> white_leds =
		{
				Pin{WHITE_LED_GPIO_Port, WHITE_LED_Pin}
		};

        m_white_leds.at(0) = white_leds.at(0);

        for (int i = 0; i < 2; ++i) {
        	m_heaters.at(i) = Heater(diag_temp_sensors[i], heater_pins[i]);
        }

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        fdcan_bus.start();
        HAL_TIM_Base_Start_IT(&htim2);
        HAL_TIM_Base_Start_IT(&htim3);
        event_loop();
    }

    void handleSensors() {
    	SensorData temp_data = {.id = static_cast<std::uint8_t>(ScienceDataID::TEMPERATURE), .data = 0};
		SensorData humidity_data = {.id = static_cast<std::uint8_t>(ScienceDataID::HUMIDITY), .data = 0};
		SensorData oxygen_data = {.id = static_cast<std::uint8_t>(ScienceDataID::OXYGEN), .data = 0};
		SensorData uv_data = {.id = static_cast<std::uint8_t>(ScienceDataID::UV), .data = 0};

		th_sensor.update_temp_humidity();
		temp_data.data = th_sensor.get_current_temp();
		humidity_data.data = th_sensor.get_current_humidity();
		oxygen_data.data = oxygen_sensor.update_oxygen();
		uv_data.data = uv_sensor.update_uv_blocking();

		science_out = temp_data;
		fdcan_bus.broadcast(science_out, SCIENCE_BOARD_ID, JETSON_ADDRESS);
		science_out = humidity_data;
		fdcan_bus.broadcast(science_out, SCIENCE_BOARD_ID, JETSON_ADDRESS);
		science_out = oxygen_data;
		fdcan_bus.broadcast(science_out, SCIENCE_BOARD_ID, JETSON_ADDRESS);
		science_out = uv_data;
		fdcan_bus.broadcast(science_out, SCIENCE_BOARD_ID, JETSON_ADDRESS);
    }

    void handleHeaterTemps() {
    	for (size_t i = 0; i < m_heaters.size(); i++) {
			m_heaters.at(i).update_temp_and_auto_shutoff_if_applicable();
		}
    }

    void feed(EnableScienceDeviceCommand const& message) {
        switch (message.science_device) {
            case ScienceDevice::HEATER_0:
                m_heaters.at(0).enable_if_possible(message.enable);
                break;
            case ScienceDevice::HEATER_1:
            	m_heaters.at(1).enable_if_possible(message.enable);
                break;
            case ScienceDevice::WHITE_LED:
                m_white_leds.at(0).write(message.enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
                break;
        }
    }

    void feed(HeaterAutoShutOffCommand const& message) {
        for (int i = 0; i < m_heaters.size(); ++i) {
            m_heaters.at(i).set_auto_shutoff(message.enable_auto_shutoff);
        }
    }

    void feed(ConfigThermistorAutoShutOffCommand const& message) {
        for (int i = 0; i < m_heaters.size(); ++i) {
            m_heaters.at(i).change_shutoff_temp(message.shutoff_temp);
        }
    }

    void receive_message() {
		std::optional<std::pair<FDCAN_RxHeaderTypeDef, InBoundScienceMessage>> received = fdcan_bus.receive();
		if (!received) Error_Handler(); // This function is called WHEN we receive a message so this should never happen

		auto const& [header, message] = received.value();

		auto messageId = std::bit_cast<FDCAN<InBoundScienceMessage>::MessageId>(header.Identifier);

		if (messageId.destination == SCIENCE_BOARD_ID) {
			std::visit([&](auto const& command) { feed(command); }, message);
		}
	}
} // namespace mrover

extern "C" {

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if (htim == &htim2) {
		mrover::handleSensors(); // gets stuck here
	} else if (htim == &htim3){
		mrover::handleHeaterTemps();
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
        mrover::receive_message();
    } else {
        // Mailbox is full OR we lost a frame
        Error_Handler();
    }
}

void HAL_PostInit() {
    mrover::init();
}

}
