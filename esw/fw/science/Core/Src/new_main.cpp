#include "new_main.h"
#include "hardware.hpp"
#include "hardware_adc.hpp"
#include "UVSensor.hpp"
#include "MethaneSensor.hpp"
#include "OxygenSensor.hpp"
#include "TempHumiditySensor.hpp"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern FDCAN_HandleTypeDef hfdcan1;

#define JETSON_ADDRESS 0x10
#define SCIENCE_A 0x50
#define SCIENCE_B 0x51

mrover::ADCSensor adc_sensor1 = mrover::ADCSensor(&hadc1, 0);
mrover::ADCSensor adc_sensor2 = mrover::ADCSensor(&hadc2, 0);
mrover::UVSensor uv_sensor = mrover::UVSensor(&adc_sensor1, 0);
mrover::MethaneSensor methane_sensor = mrover::MethaneSensor(&adc_sensor2, 0);
mrover::TempHumiditySensor th_sensor = mrover::TempHumiditySensor(&hi2c1);
mrover::OxygenSensor oxygen_sensor = mrover::OxygenSensor(&hi2c2);
mrover::FDCAN<mrover::InBoundMessage> fdcan_bus(&hfdcan1);

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
//
//}

void new_main(){
	HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 13, 1);
	HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1);

	fdcan_bus.start();

	mrover::SensorData temp_data = {1, 0};
	mrover::SensorData humidity_data = {2, 0};
	mrover::SensorData oxygen_data = {3, 0};
	mrover::SensorData methane_data = {4, 0};
	mrover::SensorData uv_data = {5, 0};

	while(1){
		th_sensor.update_temp_humidity();
		temp_data.data = th_sensor.get_current_temp();
		humidity_data.data = th_sensor.get_current_humidity();
		oxygen_data.data = oxygen_sensor.update_oxygen();
		methane_data.data = methane_sensor.update_ppm_blocking();
		uv_data.data = uv_sensor.update_uv_blocking();

		fdcan_bus.broadcast(temp_data, SCIENCE_A, JETSON_ADDRESS);
		HAL_Delay(50);
		fdcan_bus.broadcast(humidity_data, SCIENCE_A, JETSON_ADDRESS);
		HAL_Delay(50);
		fdcan_bus.broadcast(oxygen_data, SCIENCE_A, JETSON_ADDRESS);
		HAL_Delay(50);
		fdcan_bus.broadcast(methane_data, SCIENCE_A, JETSON_ADDRESS);
		HAL_Delay(50);
		fdcan_bus.broadcast(uv_data, SCIENCE_A, JETSON_ADDRESS);
		HAL_Delay(50);
	}
}
