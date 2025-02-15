#include "MethaneSensor.hpp"
#include "OxygenSensor.hpp"
#include "TempHumiditySensor.hpp"
//#include "UVSensor.hpp"
#include "UVi2c.hpp"
#include "hardware.hpp"
#include "hardware_adc.hpp"
#include "messaging_science.hpp"

//extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
extern FDCAN_HandleTypeDef hfdcan1;

#define JETSON_ADDRESS 0x10
#define SCIENCE_A 0x50
#define SCIENCE_B 0x51

namespace mrover {


//    ADCSensor adc_sensor1 = ADCSensor(&hadc1, 0);
    ADCSensor adc_sensor2 = ADCSensor(&hadc2, 0);
//    UVSensor uv_sensor = mrover::UVSensor(&adc_sensor1, 0);
    UVSensor uv_sensor = mrover::UVSensor(&hi2c3);
    MethaneSensor methane_sensor = mrover::MethaneSensor(&adc_sensor2, 0);
    TempHumiditySensor th_sensor = mrover::TempHumiditySensor(&hi2c1);
    OxygenSensor oxygen_sensor = mrover::OxygenSensor(&hi2c2);
    FDCAN<InBoundScienceMessage> fdcan_bus(&hfdcan1);

    void event_loop() {
        SensorData temp_data = {.id = static_cast<std::uint8_t>(ScienceDataID::TEMPERATURE), .data = 0};
        SensorData humidity_data = {.id = static_cast<std::uint8_t>(ScienceDataID::HUMIDITY), .data = 0};
        SensorData oxygen_data = {.id = static_cast<std::uint8_t>(ScienceDataID::OXYGEN), .data = 0};
        SensorData uv_data = {.id = static_cast<std::uint8_t>(ScienceDataID::UV), .data = 0};

        uv_sensor.init();

        while (true) {
//            th_sensor.update_temp_humidity();
//            temp_data.data = th_sensor.get_current_temp();
//            humidity_data.data = th_sensor.get_current_humidity();
//            oxygen_data.data = oxygen_sensor.update_oxygen();
//            methane_data.data = methane_sensor.update_ppm_blocking();
//            uv_data.data = uv_sensor.update_uv_blocking();
            uv_data.data = uv_sensor.update_uv();

            fdcan_bus.broadcast(temp_data, SCIENCE_A, JETSON_ADDRESS);
            HAL_Delay(50);
            fdcan_bus.broadcast(humidity_data, SCIENCE_A, JETSON_ADDRESS);
            HAL_Delay(50);
            fdcan_bus.broadcast(oxygen_data, SCIENCE_A, JETSON_ADDRESS);
            HAL_Delay(50);
            fdcan_bus.broadcast(uv_data, SCIENCE_A, JETSON_ADDRESS);
            HAL_Delay(50);
        }
    }

    void init() {
        HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 13, 1);
        HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1);

        fdcan_bus.start();
        event_loop();
    }

} // namespace mrover

void HAL_PostInit() {
    mrover::init();
}
