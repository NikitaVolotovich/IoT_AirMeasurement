#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include "main.h"
#include "bme680_defs.h"
#include "bme680.h"

int8_t bme680_start(struct bme680_dev *dev);
void bme680_refresh_data(struct bme680_dev *dev, struct bme680_field_data *data);

int8_t custom_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t custom_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);

float BME680_read_altitude(float atm);
uint8_t bme680_calculate_iaq(struct bme680_field_data data);
