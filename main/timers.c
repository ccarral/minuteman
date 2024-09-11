#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "minuteman.h"
#include "sdkconfig.h"

void ticker(TimerHandle_t xTimer) {
  minuteman_t *dev = (minuteman_t *)pvTimerGetTimerID(xTimer);
  if (xSemaphoreTake(dev->mutex, 0) == pdTRUE) {
    time(&dev->current_time);
    minuteman_alarm_check_active(dev, ALARM_0);
    minuteman_alarm_check_active(dev, ALARM_1);
    xSemaphoreGive(dev->mutex);
    xTaskNotifyGive(dev->render_task_handle);
  }
}

void toggle_display(TimerHandle_t xTimer) {
  minuteman_t *dev = (minuteman_t *)pvTimerGetTimerID(xTimer);
  if (xSemaphoreTake(dev->mutex, 0) == pdTRUE) {
    dev->display_on = !(dev->display_on);
    ESP_LOGI(__FUNCTION__, "display on: %d", dev->display_on);
    xSemaphoreGive(dev->mutex);
  }
  xTaskNotifyGive(dev->render_task_handle);
}
