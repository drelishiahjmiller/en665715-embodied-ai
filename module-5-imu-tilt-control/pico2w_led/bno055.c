#include "bno055.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

#define BNO055_ADDRESS 0x28
#define BNO055_CHIP_ID_REG 0x00
#define BNO055_CHIP_ID 0xA0
#define BNO055_PAGE_ID_REG 0x07
#define BNO055_EULER_PITCH_REG 0x1E
#define BNO055_UNIT_SEL_REG 0x3B
#define BNO055_POWER_MODE_REG 0x3E
#define BNO055_OPR_MODE_REG 0x3D
#define BNO055_CONFIG_MODE 0x00
#define BNO055_NDOF_MODE 0x0C

#define BNO055_SDA_PIN 0
#define BNO055_SCL_PIN 1
#define BNO055_I2C_TIMEOUT_US 10000

static bool write_register(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return i2c_write_timeout_us(
               i2c0,
               BNO055_ADDRESS,
               data,
               sizeof(data),
               false,
               BNO055_I2C_TIMEOUT_US) == (int) sizeof(data);
}

static bool read_registers(uint8_t reg, uint8_t *data, size_t length) {
    if (i2c_write_timeout_us(
            i2c0,
            BNO055_ADDRESS,
            &reg,
            1,
            true,
            BNO055_I2C_TIMEOUT_US) != 1) {
        return false;
    }
    return i2c_read_timeout_us(
               i2c0,
               BNO055_ADDRESS,
               data,
               length,
               false,
               BNO055_I2C_TIMEOUT_US) == (int) length;
}

bool bno055_init(void) {
    i2c_init(i2c0, 100000);
    gpio_set_function(BNO055_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(BNO055_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(BNO055_SDA_PIN);
    gpio_pull_up(BNO055_SCL_PIN);
    sleep_ms(700);

    uint8_t chip_id = 0;
    if (!read_registers(BNO055_CHIP_ID_REG, &chip_id, 1) ||
        chip_id != BNO055_CHIP_ID) {
        printf("BNO055 not found; chip ID was 0x%02X\n", chip_id);
        return false;
    }

    if (!write_register(BNO055_OPR_MODE_REG, BNO055_CONFIG_MODE)) {
        return false;
    }
    sleep_ms(25);

    if (!write_register(BNO055_PAGE_ID_REG, 0) ||
        !write_register(BNO055_POWER_MODE_REG, 0) ||
        !write_register(BNO055_UNIT_SEL_REG, 0) ||
        !write_register(BNO055_OPR_MODE_REG, BNO055_NDOF_MODE)) {
        return false;
    }
    sleep_ms(20);
    return true;
}

bool bno055_read_pitch(float *degrees) {
    if (degrees == NULL) {
        return false;
    }

    uint8_t data[2];
    if (!read_registers(BNO055_EULER_PITCH_REG, data, sizeof(data))) {
        return false;
    }

    int16_t raw = (int16_t)(((uint16_t) data[1] << 8) | data[0]);
    *degrees = (float) raw / 16.0f;
    return true;
}
