// Standard library includes
#include <string.h>
#include <stdio.h>

// Esp-idf component includes
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/i2c.h"
#include "driver/gpio.h"

#include "u8g2.h"
#include "u8g2_esp32_hal.h"

#include "ds3231.hpp"

#include<string>

static const char *TAG = "SANDBOX";

typedef struct {
	i2c_port_t port;	// I2C port number
	uint8_t addr;		// I2C address
	gpio_num_t sda_io_num;	// GPIO number for I2C sda signal
	gpio_num_t scl_io_num;	// GPIO number for I2C scl signal
	uint32_t clk_speed;		// I2C clock frequency for master mode
} i2c_dev_t;

// i2c
const gpio_num_t i2c_sda_pin = GPIO_NUM_22;
const gpio_num_t i2c_scl_pin = GPIO_NUM_21;

#define I2C_FREQ_HZ 400000
#define I2CDEV_TIMEOUT 1000

#define GPIO_INPUT_IO GPIO_NUM_19
#define GPIO_INPUT_PIN_SEL  1ULL<<GPIO_INPUT_IO
#define ESP_INTR_FLAG_DEFAULT 0

u8g2_t g_u8g2;

static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level((gpio_num_t)io_num));
        }
    }
}

uint8_t bcd2dec(uint8_t val)
{
	return (val >> 4) * 10 + (val & 0x0f);
}

esp_err_t i2c_dev_read(const i2c_dev_t *dev, const void *out_data, size_t out_size, void *in_data, size_t in_size)
{
	if (!dev || !in_data || !in_size) return ESP_ERR_INVALID_ARG;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	if (out_data && out_size)
	{
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (dev->addr << 1) | I2C_MASTER_WRITE, true);
		i2c_master_write(cmd, (const uint8_t *)out_data, out_size, true);
	}
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev->addr << 1) | 1, true);
	i2c_master_read(cmd, (uint8_t *)in_data, in_size, I2C_MASTER_LAST_NACK);
	i2c_master_stop(cmd);

	esp_err_t res = i2c_master_cmd_begin(dev->port, cmd, I2CDEV_TIMEOUT / portTICK_PERIOD_MS);
	if (res != ESP_OK)
		ESP_LOGE(TAG, "Could not read from device [0x%02x at %d]: %d", dev->addr, dev->port, res);
	i2c_cmd_link_delete(cmd);

	return res;
}

esp_err_t i2c_dev_read_reg(const i2c_dev_t *dev, uint8_t reg, void *in_data, size_t in_size)
{
	return i2c_dev_read(dev, &reg, 1, in_data, in_size);
}

esp_err_t ds3231_get_time(i2c_dev_t *dev, struct tm *time)
{
	uint8_t data[7];

	/* read time */
	esp_err_t res = i2c_dev_read_reg(dev, DS3231_ADDR_TIME, data, 7);
		if (res != ESP_OK) return res;

	/* convert to unix time structure */
	time->tm_sec = bcd2dec(data[0]);
	time->tm_min = bcd2dec(data[1]);
	if (data[2] & DS3231_12HOUR_FLAG)
	{
		/* 12H */
		time->tm_hour = bcd2dec(data[2] & DS3231_12HOUR_MASK) - 1;
		/* AM/PM? */
		if (data[2] & DS3231_PM_FLAG) time->tm_hour += 12;
	}
	else time->tm_hour = bcd2dec(data[2]); /* 24H */
	time->tm_wday = bcd2dec(data[3]) - 1;
	time->tm_mday = bcd2dec(data[4]);
	time->tm_mon  = bcd2dec(data[5] & DS3231_MONTH_MASK) - 1;
	time->tm_year = bcd2dec(data[6]) + 2000;
	time->tm_isdst = 0;

	// apply a time zone (if you are not using localtime on the rtc or you want to check/apply DST)
	//applyTZ(time);

	return ESP_OK;
}

void init_i2c() {
    i2c_config_t conf;
    memset(&conf, 0, sizeof(i2c_config_t));
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = i2c_sda_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = i2c_scl_pin;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;

    conf.master.clk_speed = I2C_FREQ_HZ;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_1, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_1, conf.mode, 0, 0, 0));
}

esp_err_t i2c_dev_init(i2c_port_t port, int sda, int scl)
{
    i2c_config_t i2c_config;
    memset(&i2c_config, 0, sizeof(i2c_config_t));
    i2c_config.mode = I2C_MODE_MASTER;
    i2c_config.sda_io_num = sda;
    i2c_config.scl_io_num = scl;
    i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.master.clk_speed = I2C_FREQ_HZ;
    i2c_param_config(I2C_NUM_1, &i2c_config);
    return i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0);
}

esp_err_t ds3231_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
	dev->port = port;
	dev->addr = DS3231_ADDR;
	dev->sda_io_num = sda_gpio;
	dev->scl_io_num = scl_gpio;
	dev->clk_speed = I2C_FREQ_HZ;
	return i2c_dev_init(port, sda_gpio, scl_gpio);
}

std::string rtcinfo_to_string(tm& rtcinfo) {
    char buffer[50];
    snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d:%02d", rtcinfo.tm_year, rtcinfo.tm_mon + 1,
                 rtcinfo.tm_mday, rtcinfo.tm_hour, rtcinfo.tm_min, rtcinfo.tm_sec);
    return std::string(buffer);
}

extern "C" void app_main(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = (gpio_pullup_t)0;
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_INPUT_IO, gpio_isr_handler, (void*) GPIO_INPUT_IO);

    i2c_dev_t dev;
    if (ds3231_init_desc(&dev, I2C_NUM_1, i2c_sda_pin, i2c_scl_pin) != ESP_OK)
    {
        ESP_LOGE(pcTaskGetName(0), "Could not init device descriptor.");
        while (1)
        {
            vTaskDelay(1);
        }
    }
    // init_i2c();
    u8g2_t *u8g2 = &g_u8g2;

    memset(u8g2, 0, sizeof(u8g2_t)); // Must zero-initialize the u8g2 struct, causes bugs otherwise

    // fixme: i2c needs to be initialized earlier; it's used for the battery charger and gas gauge also
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.sda = U8G2_ESP32_HAL_UNDEFINED;
    u8g2_esp32_hal.scl = U8G2_ESP32_HAL_UNDEFINED;
    u8g2_esp32_hal_init(u8g2_esp32_hal);

    u8g2_Setup_ssd1306_i2c_128x64_buydisplay_f(u8g2, U8G2_R0, u8g2_esp32_i2c_byte_cb, u8g2_esp32_gpio_and_delay_cb);

    u8g2_InitDisplay(u8g2);
    u8g2_SetPowerSave(u8g2, 0);
    u8g2_SetFont(u8g2, u8g2_font_6x10_tr);
    u8g2_SetFlipMode(u8g2, 0);
    u8g2_SetDrawColor(u8g2, 2);

    u8g2_ClearBuffer(u8g2);
    u8g2_SendBuffer(u8g2);

    while (1)
    {
        struct tm rtcinfo;
        std::string rtcinfo_str;
        if (ds3231_get_time(&dev, &rtcinfo) != ESP_OK)
        {
            ESP_LOGE(pcTaskGetName(0), "Could not get time.");
            while (1)
            {
                vTaskDelay(1);
            }
        }
        ESP_LOGI(pcTaskGetName(0), "%04d-%02d-%02d %02d:%02d:%02d",
                 rtcinfo.tm_year, rtcinfo.tm_mon + 1,
                 rtcinfo.tm_mday, rtcinfo.tm_hour, rtcinfo.tm_min, rtcinfo.tm_sec);

        rtcinfo_str = rtcinfo_to_string(rtcinfo);
        
        u8g2_ClearBuffer(u8g2);
        u8g2_DrawStr(u8g2, 3, 23, "Madonna Electronics");
        u8g2_DrawStr(u8g2, 3, 50, rtcinfo_str.c_str());
        u8g2_SendBuffer(u8g2);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
