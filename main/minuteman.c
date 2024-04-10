/* 
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_err.h"
#include <time.h>
#include <sys/time.h>
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_event.h"
#include "wifi.h"
#include "esp_sntp.h"
#include <max7219.h>
#include <encoder.h>
#include <minuteman.h>

#define PIN_NUM_MOSI CONFIG_PIN_NUM_MOSI
#define PIN_NUM_CLK CONFIG_PIN_NUM_CLK
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 0, 0)
#define HOST    HSPI_HOST
#else
#define HOST    SPI2_HOST
#endif

#define PIN_NUM_CS CONFIG_PIN_NUM_CS
#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)

esp_err_t minuteman_render_display(minuteman_t* dev){
    if (xSemaphoreTake(dev->mutex, 0) == pdTRUE){
        CHECK(max7219_clear(dev->display));
        if(!dev->display_on){
            sprintf(dev->digits, "%s", "");
        }else{
            switch(dev->state){
                case CLOCK_MODE:
                    // Ticker has already updated display, do nothing
                    break;
                case ALARM_EDIT:
                    localtime_r(&dev->alarms[dev->selected_alarm_idx], &dev->timeinfo);
                    strftime(dev->digits, sizeof(dev->digits), "00%H%M%S", &dev->timeinfo);
                  break;
            }
        }
        CHECK(max7219_draw_text_7seg(dev->display, 0, dev->digits));
        ESP_LOGI(__FUNCTION__, "Display updated");
        xSemaphoreGive(dev->mutex);
    }
    return ESP_OK;
}

esp_err_t display_init(max7219_t *display){
    spi_bus_config_t cfg = {
       .mosi_io_num = PIN_NUM_MOSI,
       .miso_io_num = -1,
       .sclk_io_num = PIN_NUM_CLK,
       .quadwp_io_num = -1,
       .quadhd_io_num = -1,
       .max_transfer_sz = 0,
       .flags = 0
    };

    CHECK(spi_bus_initialize(HOST, &cfg, 1));

    // Configure device
    display->cascade_size = 1;
    display->digits = 8;
    display->mirrored = true;

    CHECK(max7219_init_desc(display, HOST, MAX7219_MAX_CLOCK_SPEED_HZ, PIN_NUM_CS));
    CHECK(max7219_init(display));
    return ESP_OK;
}

esp_err_t minuteman_init(minuteman_t* dev){
    dev->display = (max7219_t*)malloc(sizeof(max7219_t));
    CHECK(display_init(dev->display));
    dev->state = CLOCK_MODE;
    sprintf(dev->digits, "00000000");
    /* TODO: Check for errors on mutex create */
    dev->mutex = xSemaphoreCreateMutex();
    dev->alarms[0] = 0;
    dev->alarms[1] = 0;
    dev->selected_alarm_idx = 0;
    dev->display_on = true;
    return ESP_OK;
}
