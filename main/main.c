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

static QueueHandle_t re_event_queue;
static rotary_encoder_t encoder;
static button_t button_alarm0;
static button_t button_alarm1;

// TODO: Set in config
#define ENCODER_INPUT_SEC_MULTIPLIER 300

static TimerHandle_t ticker_timer;
static TimerHandle_t return_to_clock_timer;
static TimerHandle_t alarm_disable_timer;
static TimerHandle_t toggle_display_timer;
static minuteman_t minuteman_dev;
static TaskHandle_t render_task = NULL;
static TaskHandle_t alarm_handler_task;

bool minuteman_check_active_alarm(minuteman_t* dev, int alarm_idx){
    struct tm alarm_timeinfo = {0};
    minuteman_alarm_event_t ev;
    ev.alarm_idx = alarm_idx;
    localtime_r(&dev->current_time, &dev->timeinfo);
    localtime_r(&dev->alarms[alarm_idx].timeval, &alarm_timeinfo);
    if( dev->alarms[alarm_idx].enabled 
        && dev->timeinfo.tm_hour == alarm_timeinfo.tm_hour
        && dev->timeinfo.tm_min == alarm_timeinfo.tm_min 
        && dev->timeinfo.tm_sec == alarm_timeinfo.tm_sec
    ){
        ev.type = MINUTEMAN_ALARM_ACTIVE;
        xQueueSendToBack(alarm_event_queue, &ev, 0);
        return true;
    }
    return false;
}

static void ticker(TimerHandle_t xTimer){
    if (xSemaphoreTake(minuteman_dev.mutex, 0) == pdTRUE){
        time(&minuteman_dev.current_time);
        minuteman_check_active_alarm(&minuteman_dev, ALARM_0);
        minuteman_check_active_alarm(&minuteman_dev, ALARM_1);
        xSemaphoreGive(minuteman_dev.mutex);
        xTaskNotifyGive(render_task);
    }
}

static void disable_alarm(TimerHandle_t xTimer){
    minuteman_alarm_event_t ev;
    ev.type = MINUTEMAN_ALARM_DISABLED;
    ev.alarm_idx = ALARM_ANY;
    xQueueSendToBackFromISR(alarm_event_queue,&ev, NULL);
}

static void alarm_handler(void* arg){
    // TODO: Refactor into event queue
    minuteman_alarm_event_t e;
    while(1){
        xQueueReceive(alarm_event_queue, &e, portMAX_DELAY);
        ESP_LOGI(__FUNCTION__, "alarm 0 handler awoken");
        if(xSemaphoreTake(minuteman_dev.mutex, portMAX_DELAY) == pdTRUE){
            switch(e.type){
                case MINUTEMAN_ALARM_ENABLED:
                    minuteman_dev.alarms[e.alarm_idx].enabled = true;
                    ESP_LOGI(__FUNCTION__, "alarm 0 enabled");
                    xTaskNotifyGive(render_task);
                    break;
                case MINUTEMAN_ALARM_ACTIVE:
                    minuteman_dev.alarms[e.alarm_idx].active = true;
                    ESP_LOGI(__FUNCTION__, "alarm 0 active");
                    xTimerReset(alarm_disable_timer, portMAX_DELAY);
                    break;
                case MINUTEMAN_ALARM_DISABLED:
                    minuteman_dev.alarms[e.alarm_idx].enabled=false;
                    ESP_LOGI(__FUNCTION__, "alarm 0 disabled");
                    xTaskNotifyGive(render_task);
                case MINUTEMAN_ALARM_INACTIVE:
                    minuteman_dev.alarms[e.alarm_idx].active = false;
                    ESP_LOGI(__FUNCTION__, "alarm 0 inactive");
                    break;
                case MINUTEMAN_ALARM_SNOOZED:
                  break;
            }
            // TODO: If snooze, create timer that will set alarm to active in SNOZE_TIME seconds
            xSemaphoreGive(minuteman_dev.mutex);
        }
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
    re_event_queue = xQueueCreate(EV_QUEUE_LEN, sizeof(rotary_encoder_event_t));

    // Setup rotary encoder library
    ESP_ERROR_CHECK(rotary_encoder_init(re_event_queue));

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
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
    esp_sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    esp_sntp_init();
}

static void render_handler(void* arg){
    ESP_LOGI(__FUNCTION__, "render task started");
    while(1){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ESP_ERROR_CHECK(minuteman_render_display(&minuteman_dev));
    }
}

// TODO: Send device via arg*
void encoder_handler(void *arg)
{

    ESP_LOGI(__FUNCTION__, "encoder task started");
    rotary_encoder_event_t e;

    while (1)
    {
        xQueueReceive(re_event_queue, &e, portMAX_DELAY);

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

static void on_button0_press(button_t *btn, button_state_t state)
{
    minuteman_alarm_event_t ev;
    ev.alarm_idx = 0;
    if(state == BUTTON_PRESSED_LONG){
        ev.type = MINUTEMAN_ALARM_ENABLED;
        xQueueSendToBack(alarm_event_queue, &ev, 0);
    }
    if(state == BUTTON_RELEASED){
        ev.type = MINUTEMAN_ALARM_DISABLED;
        xQueueSendToBack(alarm_event_queue, &ev, 0);
    }
}

static void on_button1_press(button_t *btn, button_state_t state)
{
    minuteman_alarm_event_t ev;
    ev.alarm_idx = 1;
    if(state == BUTTON_PRESSED_LONG){
        ev.type = MINUTEMAN_ALARM_ENABLED;
        xQueueSendToBack(alarm_event_queue, &ev, 0);
    }
    if(state == BUTTON_RELEASED){
        ev.type = MINUTEMAN_ALARM_DISABLED;
        xQueueSendToBack(alarm_event_queue, &ev, 0);
    }
}

esp_err_t init_alarm_enable_buttons(){
    alarm_event_queue = xQueueCreate(EV_QUEUE_LEN, sizeof(minuteman_alarm_event_t));
    /* TODO: Set GPIO ports in menuconfig */
    button_alarm0.gpio = GPIO_NUM_25;
    button_alarm0.pressed_level = 0;
    button_alarm0.internal_pull = true;
    button_alarm0.autorepeat = false;
    button_alarm0.callback = on_button0_press;
    CHECK(button_init(&button_alarm0));

    button_alarm1.gpio = GPIO_NUM_26;
    button_alarm1.pressed_level = 0;
    button_alarm1.internal_pull = true;
    button_alarm1.autorepeat = false;
    button_alarm1.callback = on_button1_press;
    CHECK(button_init(&button_alarm1));
    return ESP_OK;
}

void app_main(void)
{
    set_timezone();
    ESP_ERROR_CHECK(nvs_flash_init()); 
    initialize_sntp();
    encoder_init(&encoder);
    minuteman_init(&minuteman_dev);
    init_alarm_enable_buttons();
    ticker_timer = xTimerCreate("1000ms timer", pdMS_TO_TICKS(1000), pdTRUE, NULL, ticker);
    // TODO: Make flash interval and return to clock mode timeout compile time configs
    return_to_clock_timer = xTimerCreate("return to clock mode automatically", pdMS_TO_TICKS(5000), pdFALSE, NULL, return_to_clock_mode);
    toggle_display_timer = xTimerCreate("toggle disple on/off", pdMS_TO_TICKS(500), pdTRUE, NULL, toggle_display);
    /* TODO: Make alarm automatic disable timeout a config value */
    alarm_disable_timer = xTimerCreate("disable alarm after a while", pdMS_TO_TICKS(10000), pdFALSE, NULL, disable_alarm);
    xTimerStart(ticker_timer, portMAX_DELAY);
    xTaskCreatePinnedToCore(&alarm_handler, "alarm task", configMINIMAL_STACK_SIZE * 8, NULL, 5, &alarm_handler_task, 0);
    xTaskCreatePinnedToCore(&encoder_handler, "encoder task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL,0);
    xTaskCreatePinnedToCore(&render_handler, "render task", configMINIMAL_STACK_SIZE * 8, NULL, 5, &render_task, 0);
    for(;;);
}
