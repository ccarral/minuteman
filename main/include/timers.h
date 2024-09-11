#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

void ticker(TimerHandle_t xTimer);
void toggle_display(TimerHandle_t xTimer);
