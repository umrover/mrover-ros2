#include "main.h"

#define BME280_ADDR 0x77
#define BME280_REG_CTRL_HUM 0xF2
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_CONFIG 0xF5
#define BME280_REG_DATA 0xF7

namespace mrover {
struct THP_data {
	float temp = 0; // temp in C
	float humidity = 0; // relative humidity
	float pressure = 0; // pressure in Pa
};

class THP {
private:
	I2C_HandleTypeDef* i2c; // i2c handle pointer
	uint8_t rx_buffer[8]; // I2C receive buffer
	THP_data thp_data;

	// dig_xx variables are callibration constants, will be set during sensor initialization
	uint16_t dig_t1;
	int16_t  dig_t2, dig_t3;

	uint16_t dig_p1;
	int16_t  dig_p2, dig_p3, dig_p4, dig_p5, dig_p6, dig_p7, dig_p8, dig_p9;

	uint8_t  dig_h1, dig_h3, dig_h6;
	int16_t  dig_h2, dig_h4, dig_h5;

	// t_comp is for compensating humidity and pressure values
	int32_t t_comp;
public:
	THP() = default;

	THP (I2C_HandleTypeDef* i2c_in)
		: i2c(i2c_in) {}

	// reads and sets calibration constants
	void read_calibration() {
	    uint8_t buf1[26];
	    uint8_t buf2[7];

	    // read temp and pressure calibration
	    HAL_I2C_Mem_Read(i2c, BME280_ADDR << 1, 0x88, I2C_MEMADD_SIZE_8BIT, buf1, 26, HAL_MAX_DELAY);

	    dig_t1 = (uint16_t)(buf1[1] << 8) | buf1[0];
	    dig_t2 = (int16_t)(buf1[3] << 8) | buf1[2];
	    dig_t3 = (int16_t)(buf1[5] << 8) | buf1[4];

	    dig_p1 = (uint16_t)(buf1[7] << 8) | buf1[6];
	    dig_p2 = (int16_t)(buf1[9] << 8) | buf1[8];
	    dig_p3 = (int16_t)(buf1[11] << 8) | buf1[10];
	    dig_p4 = (int16_t)(buf1[13] << 8) | buf1[12];
	    dig_p5 = (int16_t)(buf1[15] << 8) | buf1[14];
	    dig_p6 = (int16_t)(buf1[17] << 8) | buf1[16];
	    dig_p7 = (int16_t)(buf1[19] << 8) | buf1[18];
	    dig_p8 = (int16_t)(buf1[21] << 8) | buf1[20];
	    dig_p9 = (int16_t)(buf1[23] << 8) | buf1[22];

	    // read humidity calibration
	    HAL_I2C_Mem_Read(i2c, BME280_ADDR << 1, 0xA1, I2C_MEMADD_SIZE_8BIT, &dig_h1, 1, HAL_MAX_DELAY);
	    HAL_I2C_Mem_Read(i2c, BME280_ADDR << 1, 0xE1, I2C_MEMADD_SIZE_8BIT, buf2, 7, HAL_MAX_DELAY);

	    dig_h2 = (int16_t)(buf2[1] << 8) | buf2[0];
	    dig_h3 = buf2[2];
	    dig_h4 = (int16_t)((buf2[3] << 4) | (buf2[4] & 0x0F));
	    dig_h5 = (int16_t)((buf2[5] << 4) | (buf2[4] >> 4));
	    dig_h6 = (int8_t)buf2[6];
	}

	// initializes the thp sensor
	void init() {
		// set calibration constants
		read_calibration();

		// set humidity oversampling x1
		uint8_t ctrl_hum = 0x01;
		HAL_I2C_Mem_Write(i2c, BME280_ADDR << 1, BME280_REG_CTRL_HUM, I2C_MEMADD_SIZE_8BIT, &ctrl_hum, 1, HAL_MAX_DELAY);

		// set temp oversampling to x1, pressure x1, and normal mode
		uint8_t ctrl_meas = 0x27;
		HAL_I2C_Mem_Write(i2c, BME280_ADDR << 1, BME280_REG_CTRL_MEAS, I2C_MEMADD_SIZE_8BIT, &ctrl_meas, 1, HAL_MAX_DELAY);

		// Standby 62.5 ms (close to 10 Hz), filter off
		uint8_t config = 0x20;
		HAL_I2C_Mem_Write(i2c, BME280_ADDR << 1, BME280_REG_CONFIG, I2C_MEMADD_SIZE_8BIT, &config, 1, HAL_MAX_DELAY);
	}

	// non-blocking read of the data register on the thp sensor
	void read_thp() {
		HAL_I2C_Mem_Read_IT(i2c, (BME280_ADDR << 1) | 1, BME280_REG_DATA, I2C_MEMADD_SIZE_8BIT, rx_buffer, 8);
	}

	// converts the raw temp data from the sensor to the actual temp data
	void compensate_temperature() {
	    int32_t adc_t = ((int32_t)rx_buffer[3] << 12) | ((int32_t)rx_buffer[4] << 4)  | ((int32_t)rx_buffer[5] >> 4);
	    int32_t t1 = ((((adc_t >> 3) - ((int32_t)dig_t1 << 1))) * ((int32_t)dig_t2)) >> 11;
	    int32_t t2 = (((((adc_t >> 4) - ((int32_t)dig_t1)) * ((adc_t >> 4) - ((int32_t)dig_t1))) >> 12) * ((int32_t)dig_t3)) >> 14;

	    t_comp = t1 + t2;
	    thp_data.temp = ((t_comp * 5 + 128) >> 8) / 100.0f;
	}

	// converts the raw humidity data from the sensor to the actual humidity data
	void compensate_humidity() {
	    int32_t adc_h = ((int32_t)rx_buffer[6] << 8) | (int32_t)rx_buffer[7];
	    int32_t v_x1;

	    v_x1 = t_comp - 76800;
	    v_x1 = (((((adc_h << 14) - ((int32_t)dig_h4 << 20) - ((int32_t)dig_h5 * v_x1)) + 16384) >> 15) *
	           (((((((v_x1 * (int32_t)dig_h6) >> 10) * (((v_x1 * (int32_t)dig_h3) >> 11) + 32768)) >> 10) + 2097152) * (int32_t)dig_h2 + 8192) >> 14));
	    v_x1 = v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * (int32_t)dig_h1) >> 4);

	    if (v_x1 < 0)
	    	v_x1 = 0;

	    if (v_x1 > 419430400)
	        v_x1 = 419430400;

	    thp_data.humidity = (v_x1 >> 12) / 1024.0f;
	}

	// converts the pressure raw data from the sensor to the actual pressure data
	void compensate_pressure() {
	    int32_t adc_p = ((int32_t)rx_buffer[0] << 12) | ((int32_t)rx_buffer[1] << 4)  | ((int32_t)rx_buffer[2] >> 4);

	    int64_t var1 = ((int64_t)t_comp) - 128000;
	    int64_t var2 = var1 * var1 * (int64_t)dig_p6;
	    var2 = var2 + ((var1 * (int64_t)dig_p5) << 17);
	    var2 = var2 + (((int64_t)dig_p4) << 35);
	    var1 = ((var1 * var1 * (int64_t)dig_p3) >> 8) + ((var1 * (int64_t)dig_p2) << 12);
	    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_p1) >> 33;

	    if (var1 == 0) {
	        thp_data.pressure = 0;
	        return;
	    }

	    int64_t p = 1048576 - adc_p;
	    p = (((p << 31) - var2) * 3125) / var1;
	    var1 = (((int64_t)dig_p9) * (p >> 13) * (p >> 13)) >> 25;
	    var2 = (((int64_t)dig_p8) * p) >> 19;
	    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_p7) << 4);

	    thp_data.pressure = p / 256.0f;
	}


	// update the temp, humidity, and pressure values
	THP_data update_thp() {
		compensate_temperature();
		compensate_pressure();
		compensate_humidity();

		return thp_data;
	}

	[[nodiscard]] THP_data get_thp() {
		return thp_data;
	}
};
}
