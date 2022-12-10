#include "bme680_utils.h"

uint64_t last_tick = 0;
uint8_t wait_for_data = 0;
uint8_t data_is_ready = 0;

int8_t bme680_start(struct bme680_dev *dev) {
	int8_t rslt = BME680_OK;

	dev->dev_id = BME680_I2C_ADDR_SECONDARY;
	dev->amb_temp = 20;
	dev->read = custom_i2c_read;
	dev->write = custom_i2c_write;
	dev->intf = BME680_I2C_INTF;
	dev->delay_ms = HAL_Delay;

	rslt = bme680_init(dev);

	if (rslt == BME680_OK) {
		dev->power_mode = BME680_FORCED_MODE;

		uint16_t settings_sel;

		dev->tph_sett.filter = BME680_FILTER_SIZE_7;
		dev->tph_sett.os_hum = BME680_OS_2X;
		dev->tph_sett.os_pres = BME680_OS_2X;
		dev->tph_sett.os_temp = BME680_OS_2X;

		dev->gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
		dev->gas_sett.heatr_dur = HEATR_DUR;

		settings_sel = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_GAS_SENSOR_SEL;

		dev->gas_sett.heatr_temp = HIGH_TEMP;

		rslt = bme680_set_sensor_settings(settings_sel, dev);
	}

	return rslt;
}

void bme680_refresh_data(struct bme680_dev *dev, struct bme680_field_data *data) {
	if(!wait_for_data){
		int8_t rslt = BME680_OK;

		uint16_t profile_dur = 10;
		bme680_get_profile_dur(&profile_dur, dev);

		if (rslt == BME680_OK) {
			rslt = bme680_set_sensor_mode(dev); /* Trigger a measurement */
			wait_for_data = 1;
			last_tick = HAL_GetTick();
			if(data_is_ready)
				bme680_get_sensor_data(data, dev);
		}
	} else if (wait_for_data && HAL_GetTick() - last_tick > 2500) {
		wait_for_data = 0;
		data_is_ready = 1;
	}
}

uint8_t bme680_calculate_iaq(struct bme680_field_data data){
	float humidity_score, gas_score;
	uint16_t humidity_reference = 40;
	uint32_t gas_reference = 250000;

	if (data.humidity >= 38 && data.humidity <= 42) {
		humidity_score = 0.25*100; // Humidity +/-5% around optimum
	} else { //sub-optimal
		if (data.humidity < 38) {
			humidity_score = 0.25 / humidity_reference * data.humidity * 100;
		} else {
			humidity_score = ((-0.25 / (100-humidity_reference) * data.humidity) + 0.416666) * 100;
		}
	}

	//Calculate gas contribution to IAQ index
	uint16_t gas_lower_limit = 5000;   // Bad air quality limit
	uint16_t gas_upper_limit = 50000;  // Good air quality limit

	if (gas_reference > gas_upper_limit)
		gas_reference = gas_upper_limit;
	if (gas_reference < gas_lower_limit)
		gas_reference = gas_lower_limit;

	gas_score = (0.75/(gas_upper_limit-gas_lower_limit)*gas_reference -(gas_lower_limit*(0.75/(gas_upper_limit-gas_lower_limit))))*100;

	float air_quality_score = humidity_score + gas_score;
	return (100-air_quality_score) * 5;
}

float BME680_ReadAltitude(float atm) {
    float att = 0.0f;
    att = 44330.0 * (1.0 - pow(atm / BME680_EALEVELPRESSURE_PA, 0.1903));
    return att;
}


int8_t custom_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	int8_t result;

	if (HAL_I2C_Master_Transmit(&hi2c1, (dev_id << 1), &reg_addr, 1, 10) != HAL_OK) {
		result = -1;
	} else if (HAL_I2C_Master_Receive (&hi2c1, (dev_id << 1) | 0x01, reg_data, len, 10) != HAL_OK) {
		result = -1;
	} else {
		result = 0;
	}
	return result;
}

int8_t custom_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
	int8_t result;
	int8_t *buf;

	// Allocate and load I2C transmit buffer
	buf = malloc(len + 1);
	buf[0] = reg_addr;
	memcpy(buf + 1, reg_data, len);

	if (HAL_I2C_Master_Transmit(&hi2c1, (dev_id << 1), (uint8_t *) buf, len + 1, HAL_MAX_DELAY) != HAL_OK) {
		result = -1;
	} else {
		result = 0;
	}

	free(buf);
	return result;
}
