
#include "stdint.h"
#include "i2c_device.h"

int i2c_write_bytes(I2CDevice_t i2c_device, uint8_t reg_addr, uint8_t *data, uint16_t length) {
    
    /*
    if (i2c_device == NULL || (length > 0 && data == NULL)) {
        return ESP_FAIL;
    }

    i2c_device_t* device = (i2c_device_t *)i2c_device;

    i2c_cmd_handle_t write_cmd = i2c_cmd_link_create();
    i2c_master_start(write_cmd);
    i2c_master_write_byte(write_cmd, (device->addr << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(write_cmd, reg_addr, 1);
    if (length > 0) {
        i2c_master_write(write_cmd, data, length, 1);
    }
    i2c_master_stop(write_cmd);

    esp_err_t err = ESP_FAIL;

    i2c_apply_bus(i2c_device);
    err = i2c_master_cmd_begin(device->i2c_port->port, write_cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_free_bus(i2c_device);

    i2c_cmd_link_delete(write_cmd);

    if (err != ESP_OK) {
        log_e("I2C Write Error, addr: 0x%02x, reg: 0x%02x, length: %d, Code: 0x%x", device->addr, reg_addr, length, err);
    } else {
        log_i("I2C Write Success, addr: 0x%02x, reg: 0x%02x, length: %d", device->addr, reg_addr, length);
        log_reg(data, length);
    }
    */

    return 0;
}
