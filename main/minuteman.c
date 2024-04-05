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
#include <max7219.h>
#include <encoder.h>

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 0, 0)
#define HOST    HSPI_HOST
#else
#define HOST    SPI2_HOST
#endif

#define RE_A_GPIO  CONFIG_PIN_ROT_ENCODER_A 
#define RE_B_GPIO  CONFIG_PIN_ROT_ENCODER_B 
#define RE_BTN_GPIO  CONFIG_PIN_ROT_ENCODER_BUTTON 

#define EV_QUEUE_LEN CONFIG_RE_EV_QUEUE_LEN 

static QueueHandle_t event_queue;
static rotary_encoder_t encoder;

/* Can use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO
#define PIN_NUM_MOSI CONFIG_PIN_NUM_MOSI
#define PIN_NUM_CLK CONFIG_PIN_NUM_CLK
#define PIN_NUM_CS CONFIG_PIN_NUM_CS

typedef enum{
    CLOCK_MODE,
    ALARM_EDIT
} minuteman_state_t;

typedef struct {
    char digits[9];
    minuteman_state_t state;
    max7219_t* display;
    SemaphoreHandle_t mutex;
    time_t current_time;
    time_t alarms[2];
    int selected_alarm_idx;
    bool display_on;
} minuteman_t;

static TimerHandle_t ticker_timer;
static TimerHandle_t return_to_clock_timer;
static TimerHandle_t toggle_display_timer;
static char strtime_buf[9];
static max7219_t display;
static minuteman_t device;
static TaskHandle_t render_task = NULL;

esp_err_t render_display(minuteman_t* dev){
    if (xSemaphoreTake(dev->mutex, 0) == pdTRUE){
        ESP_ERROR_CHECK(max7219_clear(dev->display));
        if(!dev->display_on){
            sprintf(dev->digits, "%s", "");
        }else{
            switch(dev->state){
                case CLOCK_MODE:
                    // Ticker has already updated display, do nothing
                    break;
                case ALARM_EDIT:
                    sprintf(dev->digits, "%08ld", dev->alarms[dev->selected_alarm_idx]);
                  break;
            }
        }
        ESP_ERROR_CHECK(max7219_draw_text_7seg(dev->display, 0, dev->digits));
        ESP_LOGI(__FUNCTION__, "Display updated");
        xSemaphoreGive(dev->mutex);
    }
    return ESP_OK;
}

static void ticker(TimerHandle_t xTimer){
    time_t now = 0;
    time(&now);
    static struct tm timeinfo = { 0 };
    localtime_r(&now, &timeinfo);
    strftime(strtime_buf, sizeof(strtime_buf), "00%I%M%S", &timeinfo);
    bool call_render = false;
    if (xSemaphoreTake(device.mutex, 0) == pdTRUE){
        device.current_time = now;
        if(device.state == CLOCK_MODE){
            ESP_LOGI(__FUNCTION__, "Time updated");
            sprintf(device.digits,"%s", strtime_buf);
            call_render = true;
        }
        xSemaphoreGive(device.mutex);
        if (call_render){
            xTaskNotifyGive(render_task);
        }
    }
}

void enter_edit_mode_timers(){
    xTimerReset(return_to_clock_timer, portMAX_DELAY);
    xTimerReset(toggle_display_timer, portMAX_DELAY);
}

static void return_to_clock_mode(TimerHandle_t xTimer){
    if (xSemaphoreTake(device.mutex, 0) == pdTRUE){
        device.state = CLOCK_MODE;
        device.display_on = true;
        xSemaphoreGive(device.mutex);
    }
    xTimerStop(toggle_display_timer, portMAX_DELAY);
    xTaskNotifyGive(render_task);
}

void setup_gpio(){
    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

void display_init(max7219_t *display){
    spi_bus_config_t cfg = {
       .mosi_io_num = PIN_NUM_MOSI,
       .miso_io_num = -1,
       .sclk_io_num = PIN_NUM_CLK,
       .quadwp_io_num = -1,
       .quadhd_io_num = -1,
       .max_transfer_sz = 0,
       .flags = 0
    };

    ESP_ERROR_CHECK(spi_bus_initialize(HOST, &cfg, 1));

    // Configure device
    display->cascade_size = 1;
    display->digits = 8;
    display->mirrored = true;

    ESP_ERROR_CHECK(max7219_init_desc(display, HOST, MAX7219_MAX_CLOCK_SPEED_HZ, PIN_NUM_CS));
    ESP_ERROR_CHECK(max7219_init(display));
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

void minuteman_init(minuteman_t* dev, max7219_t* display){
    dev->display = display;
    dev->state = CLOCK_MODE;
    sprintf(dev->digits, "00000000");
    /* TODO: Check for errors on mutex create */
    dev->mutex = xSemaphoreCreateMutex();
    dev->alarms[0] = 0;
    dev->alarms[1] = 0;
    dev->selected_alarm_idx = 0;
    dev->display_on = true;
}

static void toggle_display(TimerHandle_t xTimer){
    if(xSemaphoreTake(device.mutex, 0) == pdTRUE){
        device.display_on = !(device.display_on);
        ESP_LOGI(__FUNCTION__, "display on: %d", device.display_on);
        xSemaphoreGive(device.mutex);
    }
    xTaskNotifyGive(render_task);
}

void set_timezone(){
    setenv("TZ", "UTC-6", 1);
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
        ESP_ERROR_CHECK(render_display(&device));
    }
}

// TODO: Send device via arg*
void encoder_handler(void *arg)
{

    rotary_encoder_event_t e;
    int32_t val = 0;

    while (1)
    {
        xQueueReceive(event_queue, &e, portMAX_DELAY);

        switch (e.type)
        {
            case RE_ET_BTN_PRESSED:
                if(xSemaphoreTake(device.mutex, 0) == pdTRUE){
                    switch(device.state){
                        case CLOCK_MODE:
                            device.state = ALARM_EDIT;
                            device.selected_alarm_idx = 0;
                            enter_edit_mode_timers();
                            // TODO: Disable ticker timer
                            break;
                        case ALARM_EDIT:
                            if(device.selected_alarm_idx == 1){
                                device.state = CLOCK_MODE;
                            }else{
                                device.selected_alarm_idx += 1;
                                device.state = ALARM_EDIT;
                                enter_edit_mode_timers();
                                xTimerReset(return_to_clock_timer, portMAX_DELAY);
                            }
                            break;
                    }
                    xSemaphoreGive(device.mutex);
                    ESP_LOGI(__FUNCTION__, "render called from RE button press");
                    xTaskNotifyGive(render_task);
                }
                break;
            case RE_ET_BTN_RELEASED:
                break;
            case RE_ET_BTN_CLICKED:
                // Set to edit mode and create task that unsets it
                break;
            case RE_ET_BTN_LONG_PRESSED:
                break;
            case RE_ET_CHANGED:
                val += e.diff;
                if(xSemaphoreTake(device.mutex, 0) == pdTRUE){
                    if(device.state == ALARM_EDIT){
                        device.alarms[device.selected_alarm_idx] += e.diff;
                        xTimerReset(return_to_clock_timer, portMAX_DELAY);
                    }
                    xSemaphoreGive(device.mutex);
                    ESP_LOGI(__FUNCTION__, "render called from RE value change");
                    xTaskNotifyGive(render_task);
                }
                break;
            default:
                break;
        }
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
    set_timezone();
    setup_gpio();
    ESP_ERROR_CHECK(nvs_flash_init()); 
    initialize_sntp();
    display_init(&display);
    encoder_init(&encoder);
    minuteman_init(&device, &display);
    ticker_timer = xTimerCreate("1000ms timer", pdMS_TO_TICKS(1000), pdTRUE, NULL, ticker);
    // TODO: Make flash interval and return to clock mode timeout compile time configs
    return_to_clock_timer = xTimerCreate("return to clock mode automatically", pdMS_TO_TICKS(5000), pdFALSE, NULL, return_to_clock_mode);
    toggle_display_timer = xTimerCreate("toggle disple on/off", pdMS_TO_TICKS(500), pdTRUE, NULL, toggle_display);
    xTimerStart(ticker_timer, portMAX_DELAY);
    xTaskCreatePinnedToCore(&encoder_handler, "encoder task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL,0);
    xTaskCreatePinnedToCore(&render_handler, "render task", configMINIMAL_STACK_SIZE * 8, NULL, 5, &render_task,0);
    for(;;);
}
