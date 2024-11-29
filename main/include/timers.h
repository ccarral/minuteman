#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

void ticker(TimerHandle_t xTimer);
void toggle_display(TimerHandle_t xTimer);
void return_to_clock_mode(TimerHandle_t xTimer);
void reactivate_snoozed_alarms(TimerHandle_t xTimer);
void disable_alarm(TimerHandle_t xTimer);
void reenable_mask(TimerHandle_t xTimer);
