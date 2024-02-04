/* Blink Example

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

/* Can use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO
static const char *TAG = "LAZARUS";

static TimerHandle_t ticker_timer;
static int led_status;
static int counter;

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

void app_main(void)
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BLINK_GPIO, 1);
    led_status = 1;
    ticker_timer = xTimerCreate("1000ms timer", pdMS_TO_TICKS(1000), pdTRUE, NULL, ticker_1000ms);
    xTimerStart(ticker_timer, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(30000));
    for(;;);
}
