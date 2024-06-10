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
#include "driver/gpio.h"
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
static char buf[6];

#define TIME_FMT "%H%M%S"

esp_err_t minuteman_render_display(minuteman_t* dev){
    if (xSemaphoreTake(dev->mutex, 0) == pdTRUE){
        CHECK(max7219_clear(dev->display));
        if(!dev->display_on){
            sprintf(dev->digits, "%s", "      ");
        }else{
            switch(dev->state){
                case CLOCK_MODE:
                    localtime_r(&dev->current_time, &dev->timeinfo);
                    strftime(dev->digits, sizeof(dev->digits), TIME_FMT, &dev->timeinfo);
                    break;
                case ALARM_EDIT:
                    localtime_r(&dev->alarms[dev->selected_alarm_idx].timeval, &dev->timeinfo);
                    strftime(dev->digits, sizeof(dev->digits), TIME_FMT, &dev->timeinfo);
                  break;
            }
        }
        CHECK(max7219_draw_text_7seg(dev->display, 2, dev->digits));
        /* Buttons mean different things depending on edit mode enabled or disabled */
        // TODO: Refactor this code so the following happens:
        // When on clock mode, the alarm light is on if the alarm is enabled.
        // When on alarm edit mode, the alarm light is on if the current alarm is being edited.
        if(dev->alarms[0].enabled){
            CHECK(max7219_set_digit(dev->display, 0, 0b11111111));
        }else{
            CHECK(max7219_set_digit(dev->display, 0, 0));
        }
        if(dev->alarms[1].enabled){
            CHECK(max7219_set_digit(dev->display, 1, 0b11111111));
        }else{
            CHECK(max7219_set_digit(dev->display, 1, 0));
        }
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
    max7219_set_brightness(display, MAX7219_MAX_BRIGHTNESS/2);
    CHECK(max7219_draw_text_7seg(display, 2, "888888"));
    return ESP_OK;
}

void init_alarm(minuteman_alarm_t* alarm){
    alarm->enabled = false;
    alarm->active = false;
    alarm->snoozed = false;
    alarm->timeval = 0;
}

esp_err_t minuteman_init(minuteman_t* dev){
    dev->display = (max7219_t*)malloc(sizeof(max7219_t));
    CHECK(display_init(dev->display));
    dev->state = CLOCK_MODE;
    sprintf(dev->digits, "00000000");
    /* TODO: Check for errors on mutex create */
    dev->mutex = xSemaphoreCreateMutex();
    init_alarm(&dev->alarms[0]);
    init_alarm(&dev->alarms[1]);
    dev->display_on = true;
    return ESP_OK;
}
