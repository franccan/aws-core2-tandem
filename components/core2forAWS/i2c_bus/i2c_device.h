#include "stdint.h"

typedef void * I2CDevice_t;

int i2c_write_bytes(I2CDevice_t i2c_device, uint8_t reg_addr, uint8_t *data, uint16_t length);
