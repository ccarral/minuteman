#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include <max7219.h>
#include <time.h>

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)

typedef enum{
    CLOCK_MODE,
    ALARM_EDIT
} minuteman_state_t;


typedef struct{
    bool enabled;
    bool active;
    bool snoozed;
    time_t timeval;
}minuteman_alarm_t;

typedef struct {
    char digits[9];
    minuteman_state_t state;
    max7219_t* display;
    SemaphoreHandle_t mutex;
    time_t current_time;
    minuteman_alarm_t alarms[2];
    int selected_alarm_idx;
    bool display_on;
    struct tm timeinfo;
} minuteman_t;

esp_err_t minuteman_render_display(minuteman_t* dev);

esp_err_t minuteman_init(minuteman_t* dev);
