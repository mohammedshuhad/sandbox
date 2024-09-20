// Standard library includes
#include <string.h>
#include <stdio.h>

// Esp-idf component includes
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "driver/i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "u8g2.h"
#include "u8g2_esp32_hal.h"

// i2c
const gpio_num_t i2c_sda_pin = GPIO_NUM_22;
const gpio_num_t i2c_scl_pin = GPIO_NUM_21;

u8g2_t g_u8g2;

void init_i2c() {
    i2c_config_t conf;
    memset(&conf, 0, sizeof(i2c_config_t));
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = i2c_sda_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = i2c_scl_pin;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;

    conf.master.clk_speed = 400000;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_1, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_1, conf.mode, 0, 0, 0));
}

extern "C" void app_main(void)
{
    init_i2c();
    u8g2_t *u8g2 = &g_u8g2;
    printf("Hello, world!\n");

    memset(u8g2, 0, sizeof(u8g2_t)); // Must zero-initialize the u8g2 struct, causes bugs otherwise

    // fixme: i2c needs to be initialized earlier; it's used for the battery charger and gas gauge also
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.sda = U8G2_ESP32_HAL_UNDEFINED;
    u8g2_esp32_hal.scl = U8G2_ESP32_HAL_UNDEFINED;
    u8g2_esp32_hal_init(u8g2_esp32_hal);

    u8g2_Setup_ssd1306_i2c_128x64_buydisplay_f(u8g2, U8G2_R0, u8g2_esp32_i2c_byte_cb, u8g2_esp32_gpio_and_delay_cb);
    // u8g2_Setup_ssd1306_i2c_128x64_noname_f(u8g2, U8G2_R0, u8g2_esp32_i2c_byte_cb, u8g2_esp32_gpio_and_delay_cb);

    u8g2_InitDisplay(u8g2);
    u8g2_SetPowerSave(u8g2, 0);
    u8g2_SetFont(u8g2, u8g2_font_6x10_tr);
    u8g2_SetFlipMode(u8g2, 0);
    u8g2_SetDrawColor(u8g2, 2);

    u8g2_ClearBuffer(u8g2);
    u8g2_SendBuffer(u8g2);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    u8g2_DrawStr(u8g2, 3, 23, "Madonna Electronics");
    u8g2_SendBuffer(u8g2);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    u8g2_DrawStr(u8g2, 3, 50, "Madonna Electronics");
    u8g2_SendBuffer(u8g2);

    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
