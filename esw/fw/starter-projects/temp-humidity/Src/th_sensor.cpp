#include <cstdint>
#include <cmath>
#include "main.h"

// TODO: add sensor I2C address
#define TEMP_HUM_ADDRESS 

extern I2C_HandleTypeDef hi2c3;

namespace mrover {

I2C_HandleTypeDef* i2c;
uint8_t th_buf[6];
double temp;
double humidity;

void eventLoop() {
	// TODO: implement main loop
	while (true) {
		
	}
}

void init() {
	i2c = &hi2c3;
	eventLoop();
}

}

extern "C" {

	void HAL_PostInit() {
		mrover::init();
	}

	// TODO: implement transmit callback
	void HAL_I2C_MasterTxCpltCallback (I2C_HandleTypeDef* hi2c) {
		
	}

	// TODO: implement receive callback
	void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef* hi2c) {
		
	}

}
