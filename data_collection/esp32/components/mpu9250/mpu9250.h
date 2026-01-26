#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"

/* ================== Data ================== */

typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t mx, my, mz;
} mpu9250_data_t;

/* ================== Configuration ================== */

typedef struct {
    spi_host_device_t spi_host;   // SPI2_HOST or SPI3_HOST
    int mosi_pin;
    int miso_pin;
    int sclk_pin;
    int cs_pin;
    int clock_speed_hz;           // e.g. 1 MHz
    uint8_t gyro_config;          // FS_SEL bits (0x00,0x08,0x10,0x18)
    uint8_t accel_config;         // AFS_SEL bits (0x00,0x08,0x10,0x18)
} mpu9250_config_t;

/* ================== API ================== */

bool mpu9250_init(const mpu9250_config_t *cfg);
bool mpu9250_get_data(mpu9250_data_t *out);
