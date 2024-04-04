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

typedef struct {
    char digits[9];
    int edit_mode;
    max7219_t* display;
    SemaphoreHandle_t mutex;
} minuteman_t;

static TimerHandle_t ticker_timer;
static int led_status;
static char strtime_buf[64];
static max7219_t display;
static minuteman_t device;
static TaskHandle_t render_task = NULL;

esp_err_t render_display(minuteman_t* dev){
    if (xSemaphoreTake(dev->mutex, 0) != pdTRUE){
        return ESP_ERR_INVALID_STATE;
    }
    /* ESP_ERROR_CHECK(max7219_clear(dev->display)); */
    ESP_ERROR_CHECK(max7219_draw_text_7seg(dev->display, 0, dev->digits));
    ESP_LOGI(__FUNCTION__, "Display updated");
    xSemaphoreGive(dev->mutex);
    return ESP_OK;
}

static void print_time(TimerHandle_t xTimer){
    time_t now = 0;
    time(&now);
    static struct tm timeinfo = { 0 };
    localtime_r(&now, &timeinfo);
    strftime(strtime_buf, sizeof(strtime_buf), "%c", &timeinfo);
    /* strftime(device.digits, sizeof(device.digits), "00%I%M%S", &timeinfo); */
    /* printf("%s\n", strtime_buf); */
    /* draw_display(&device); */
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

void minuteman_init(minuteman_t* dev, max7219_t* display){
    dev->display = display;
    dev->edit_mode = 0;
    sprintf(dev->digits, "00000000");
    /* TODO: Check for errors on mutex create */
    dev->mutex = xSemaphoreCreateMutex();
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
        ESP_LOGI(__FUNCTION__, "Render task awoken");
        ESP_ERROR_CHECK(render_display(&device));
    }

}

void encoder_handler(void *arg)
{
    // Create event queue for rotary encoders
    event_queue = xQueueCreate(EV_QUEUE_LEN, sizeof(rotary_encoder_event_t));

    // Setup rotary encoder library
    ESP_ERROR_CHECK(rotary_encoder_init(event_queue));

    // Add one encoder
    memset(&encoder, 0, sizeof(rotary_encoder_t));
    encoder.pin_a = RE_A_GPIO;
    encoder.pin_b = RE_B_GPIO;
    encoder.pin_btn = RE_BTN_GPIO;
    ESP_ERROR_CHECK(rotary_encoder_add(&encoder));

    rotary_encoder_event_t e;
    int32_t val = 0;

    while (1)
    {
        xQueueReceive(event_queue, &e, portMAX_DELAY);

        switch (e.type)
        {
            case RE_ET_BTN_PRESSED:
                if(xSemaphoreTake(device.mutex, 0) == pdTRUE){
                    device.edit_mode = 1;
                    xSemaphoreGive(device.mutex);
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
                    sprintf(device.digits, "%08d", val);
                    xSemaphoreGive(device.mutex);
                }
                render_display(&device);
                xTaskNotifyGive(render_task);
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
    minuteman_init(&device, &display);
    led_status = 1;
    /* ticker_timer = xTimerCreate("1000ms timer", pdMS_TO_TICKS(1000), pdTRUE, NULL, print_time); */
    /* xTimerStart(ticker_timer, portMAX_DELAY); */
    xTaskCreatePinnedToCore(&encoder_handler, "encoder task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL,0);
    xTaskCreatePinnedToCore(&render_handler, "render task", configMINIMAL_STACK_SIZE * 8, NULL, 5, &render_task,0);
    for(;;);
}
