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
#include <encoder.h>
#include <minuteman.h>
#include <button.h>


#define RE_A_GPIO  CONFIG_PIN_ROT_ENCODER_A 
#define RE_B_GPIO  CONFIG_PIN_ROT_ENCODER_B 
#define RE_BTN_GPIO  CONFIG_PIN_ROT_ENCODER_BUTTON 

#define EV_QUEUE_LEN CONFIG_RE_EV_QUEUE_LEN 

static QueueHandle_t event_queue;
static rotary_encoder_t encoder;
static button_t btn1;

// TODO: Set in config
#define ENCODER_INPUT_SEC_MULTIPLIER 600

static TimerHandle_t ticker_timer;
static TimerHandle_t return_to_clock_timer;
static TimerHandle_t toggle_display_timer;
static minuteman_t minuteman_dev;
static TaskHandle_t render_task = NULL;

static void ticker(TimerHandle_t xTimer){
    struct tm timeinfo = { 0 };
    if (xSemaphoreTake(minuteman_dev.mutex, 0) == pdTRUE){
        time(&minuteman_dev.current_time);
        if(minuteman_dev.state == CLOCK_MODE){
            ESP_LOGI(__FUNCTION__, "Time updated");
            strftime(minuteman_dev.digits, sizeof(minuteman_dev.digits), "00%H%M%S", &timeinfo);
        }
        xSemaphoreGive(minuteman_dev.mutex);
        xTaskNotifyGive(render_task);
    }
}

void enter_edit_mode_timers(){
    xTimerStop(ticker_timer, portMAX_DELAY);
    xTimerReset(return_to_clock_timer, portMAX_DELAY);
    xTimerReset(toggle_display_timer, portMAX_DELAY);
}

void leave_edit_mode_timers(){
    xTimerReset(ticker_timer, portMAX_DELAY);
    xTimerStop(return_to_clock_timer, portMAX_DELAY);
    xTimerStop(toggle_display_timer, portMAX_DELAY);
}

static void return_to_clock_mode(TimerHandle_t xTimer){
    if (xSemaphoreTake(minuteman_dev.mutex, 0) == pdTRUE){
        minuteman_dev.state = CLOCK_MODE;
        minuteman_dev.display_on = true;
        xSemaphoreGive(minuteman_dev.mutex);
    }
    xTimerStop(toggle_display_timer, portMAX_DELAY);
    xTimerReset(ticker_timer, portMAX_DELAY);
    xTaskNotifyGive(render_task);
}


void encoder_init(rotary_encoder_t* encoder){
    // Create event queue for rotary encoders
    event_queue = xQueueCreate(EV_QUEUE_LEN, sizeof(rotary_encoder_event_t));

    // Setup rotary encoder library
    ESP_ERROR_CHECK(rotary_encoder_init(event_queue));

    // Add one encoder
    memset(encoder, 0, sizeof(rotary_encoder_t));
    encoder->pin_a = RE_A_GPIO;
    encoder->pin_b = RE_B_GPIO;
    encoder->pin_btn = RE_BTN_GPIO;
    ESP_ERROR_CHECK(rotary_encoder_add(encoder));
}

static void toggle_display(TimerHandle_t xTimer){
    if(xSemaphoreTake(minuteman_dev.mutex, 0) == pdTRUE){
        minuteman_dev.display_on = !(minuteman_dev.display_on);
        ESP_LOGI(__FUNCTION__, "display on: %d", minuteman_dev.display_on);
        xSemaphoreGive(minuteman_dev.mutex);
    }
    xTaskNotifyGive(render_task);
}

void set_timezone(){
    setenv("TZ", "CST6", 1);
    tzset();
}

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(__FUNCTION__, "Notification of a time synchronization event");
}

static void initialize_sntp(void)
{
    ESP_LOGI(__FUNCTION__, "Initializing SNTP");
    wifi_init_sta();
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_init();
}

static void render_handler(void* arg){
    ESP_LOGI(__FUNCTION__, "Render task started");
    while(1){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ESP_ERROR_CHECK(minuteman_render_display(&minuteman_dev));
    }
}

// TODO: Send device via arg*
void encoder_handler(void *arg)
{

    rotary_encoder_event_t e;

    while (1)
    {
        xQueueReceive(event_queue, &e, portMAX_DELAY);

        switch (e.type)
        {
            case RE_ET_BTN_PRESSED:
                if(xSemaphoreTake(minuteman_dev.mutex, 0) == pdTRUE){
                    minuteman_dev.display_on = true;
                    switch(minuteman_dev.state){
                        case CLOCK_MODE:
                            minuteman_dev.state = ALARM_EDIT;
                            minuteman_dev.selected_alarm_idx = 0;
                            enter_edit_mode_timers();
                            break;
                        case ALARM_EDIT:
                            if(minuteman_dev.selected_alarm_idx == 1){
                                minuteman_dev.state = CLOCK_MODE;
                                minuteman_dev.display_on = true;
                                leave_edit_mode_timers();
                            }else{
                                minuteman_dev.selected_alarm_idx += 1;
                                minuteman_dev.state = ALARM_EDIT;
                                minuteman_dev.display_on = true;
                                enter_edit_mode_timers();
                            }
                            break;
                    }
                    xSemaphoreGive(minuteman_dev.mutex);
                    ESP_LOGI(__FUNCTION__, "render called from RE button press");
                    xTaskNotifyGive(render_task);
                }
                break;
            case RE_ET_CHANGED:
                if(xSemaphoreTake(minuteman_dev.mutex, 0) == pdTRUE){
                    minuteman_dev.display_on = true;
                    if(minuteman_dev.state == ALARM_EDIT){
                        minuteman_dev.alarms[minuteman_dev.selected_alarm_idx].timeval += (e.diff * ENCODER_INPUT_SEC_MULTIPLIER);
                        enter_edit_mode_timers();
                    }
                    xSemaphoreGive(minuteman_dev.mutex);
                    ESP_LOGI(__FUNCTION__, "render called from RE value change");
                    xTaskNotifyGive(render_task);
                }
                break;
            case RE_ET_BTN_RELEASED:
            case RE_ET_BTN_CLICKED:
            case RE_ET_BTN_LONG_PRESSED:
            default:
                break;
        }
    }
}

static void on_button(button_t *btn, button_state_t state)
{
    if(state == BUTTON_PRESSED){
        if(xSemaphoreTake(minuteman_dev.mutex, 0) == pdTRUE){
            minuteman_dev.alarms[0].enabled = true;
            xSemaphoreGive(minuteman_dev.mutex);
            xTaskNotifyGive(render_task);
        }
    }
    if(state == BUTTON_RELEASED){
        if(xSemaphoreTake(minuteman_dev.mutex, 0) == pdTRUE){
            minuteman_dev.alarms[0].enabled = false;
            xSemaphoreGive(minuteman_dev.mutex);
            xTaskNotifyGive(render_task);
        }
    }
}
void init_button(){
    btn1.gpio = GPIO_NUM_25;
    btn1.pressed_level = 0;
    btn1.internal_pull = true;
    btn1.autorepeat = false;
    btn1.callback = on_button;
    ESP_ERROR_CHECK(button_init(&btn1));
}

void app_main(void)
{
    set_timezone();
    ESP_ERROR_CHECK(nvs_flash_init()); 
    initialize_sntp();
    encoder_init(&encoder);
    minuteman_init(&minuteman_dev);
    init_button();
    ticker_timer = xTimerCreate("1000ms timer", pdMS_TO_TICKS(1000), pdTRUE, NULL, ticker);
    // TODO: Make flash interval and return to clock mode timeout compile time configs
    return_to_clock_timer = xTimerCreate("return to clock mode automatically", pdMS_TO_TICKS(5000), pdFALSE, NULL, return_to_clock_mode);
    toggle_display_timer = xTimerCreate("toggle disple on/off", pdMS_TO_TICKS(500), pdTRUE, NULL, toggle_display);
    xTimerStart(ticker_timer, portMAX_DELAY);
    xTaskCreatePinnedToCore(&encoder_handler, "encoder task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL,0);
    xTaskCreatePinnedToCore(&render_handler, "render task", configMINIMAL_STACK_SIZE * 8, NULL, 5, &render_task,0);
    for(;;);
}
