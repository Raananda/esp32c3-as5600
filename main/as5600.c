#include "as5600.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define TAG "AS5600"

#define I2C_MASTER_SCL_IO 7
#define I2C_MASTER_SDA_IO 6
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

#define AS5600_ADDR 0x36
#define RAW_ANGLE_REGISTER_HIGH 0x0C

void as5600_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

uint16_t as5600_read_raw_angle(void)
{
    uint8_t data[2];
    uint8_t reg = RAW_ANGLE_REGISTER_HIGH;

    esp_err_t ret = i2c_master_write_read_device(
        I2C_MASTER_NUM, AS5600_ADDR, &reg, 1, data, 2, 1000 / portTICK_PERIOD_MS
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(ret));
        return 0;
    }

    return (data[0] << 8) | data[1];
}

int as5600_raw_to_degrees(uint16_t raw)
{
    return (raw * 360) / 4096;
}
