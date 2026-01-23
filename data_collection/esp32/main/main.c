#include "mpu9250.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "APP"

void app_main(void)
{
    mpu9250_config_t cfg = {
        .spi_host = SPI2_HOST,
        .mosi_pin = 23,
        .miso_pin = 19,
        .sclk_pin = 18,
        .cs_pin   = 5,
        .clock_speed_hz = 1 * 1000 * 1000,
        .gyro_config  = 0x08,   // ±500 dps
        .accel_config = 0x08    // ±4g
    };

    if (!mpu9250_init(&cfg)) {
        ESP_LOGE(TAG, "MPU init failed");
        return;
    }

    mpu9250_data_t d;

    while (1) {
        if (mpu9250_get_data(&d)) {
            ESP_LOGI(TAG,
                "ACC[%d %d %d] GYR[%d %d %d] MAG[%d %d %d]",
                d.ax, d.ay, d.az,
                d.gx, d.gy, d.gz,
                d.mx, d.my, d.mz
            );
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
