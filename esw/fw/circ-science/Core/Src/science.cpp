#include <cstdint>
#include <vector>
#include "hardware.hpp"
#include "messaging_science.hpp"

extern UART_HandleTypeDef huart1;
extern FDCAN_HandleTypeDef hfdcan1;

#define JETSON_ADDRESS 0x10
#define SCIENCE_BOARD_ID 0x50

uint8_t rx_byte;

namespace mrover {

FDCAN<SensorData> fdcan_bus;
SensorData science_out;
std::vector<char> data_vec;

void eventLoop() {
	while (true) {}
}

void init() {
	fdcan_bus.start();
	HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
	eventLoop();
}

void handleByte(char byte) {
	if (byte == '\n') {
		uint64_t geiger_data = 0;
		for (size_t i = 0; i < data_vec.size(); i++) {
			uint8_t dec_value = data_vec[i] - '0';
			geiger_data = (geiger_data * 10) + dec_value;
		}
		data_vec.clear();
		science_out.id = static_cast<uint8_t>(ScienceDataID::GEIGER);
		science_out.data = geiger_data;
		fdcan_bus.broadcast(science_out, SCIENCE_BOARD_ID, JETSON_ADDRESS);
	} else {
		data_vec.push_back(byte);
	}
}

} // namespace mrover

extern "C" {

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
//{
//	if (htim == &htim2) {
//		mrover::handleI2CSensors();
//	} else if (htim == &htim3){
//		mrover::handleHeaterTemps();
//	} else if (htim == &htim4) {
//		mrover::handleAnalogSensors();
//	}
//}


void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
	FDCAN_ProtocolStatusTypeDef protocolStatus = {};
	HAL_FDCAN_GetProtocolStatus(hfdcan, &protocolStatus);
	if (protocolStatus.BusOff) {
		CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
    	mrover::handleByte(rx_byte);
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1); // restart rx interrupt
    }
}

void HAL_PostInit() {
    mrover::init();
}

}
