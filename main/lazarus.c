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
/* #include "protocol_examples_common.h" */
/* #include "nvs_flash.h" */

/* Can use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

static TimerHandle_t ticker_timer;
static int led_status;
static char strtime_buf[64];

static void ticker_1000ms(TimerHandle_t xTimer){
    switch (led_status){
        case 0:
            gpio_set_level(BLINK_GPIO, 1);
            led_status = 1;
            printf("Turning on the LED\n");
            break;
        case 1:
        default:
            gpio_set_level(BLINK_GPIO, 0);
            led_status = 0;
            printf("Turning off the LED\n");
            break;
    }

}

static void print_time(TimerHandle_t xTimer){
    time_t now = 0;
    time(&now);
    static struct tm timeinfo = { 0 };
    localtime_r(&now, &timeinfo);
    strftime(strtime_buf, sizeof(strtime_buf), "%c", &timeinfo);
    printf("%s\n", strtime_buf);
}

void set_timezone(){
    setenv("TZ", "CST-6", 1);
    tzset();
}

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(__FUNCTION__, "Notification of a time synchronization event");
}

static void initialize_sntp(void)
{
    ESP_LOGI(__FUNCTION__, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_init();
}

void app_main(void)
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    set_timezone();
    gpio_pad_select_gpio(BLINK_GPIO);
    ESP_ERROR_CHECK(nvs_flash_init()); 
    wifi_init_sta();
    initialize_sntp();
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BLINK_GPIO, 1);
    led_status = 1;
    ticker_timer = xTimerCreate("1000ms timer", pdMS_TO_TICKS(1000), pdTRUE, NULL, print_time);
    xTimerStart(ticker_timer, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(30000));
    for(;;);
}
