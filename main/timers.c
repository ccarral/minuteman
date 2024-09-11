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

void return_to_clock_mode(TimerHandle_t xTimer) {
  minuteman_t *dev = (minuteman_t *)pvTimerGetTimerID(xTimer);
  if (xSemaphoreTake(dev->mutex, 0) == pdTRUE) {
    dev->state = CLOCK_MODE;
    dev->display_on = true;
    xSemaphoreGive(dev->mutex);
  }
  xTimerStop(dev->toggle_display_timer, portMAX_DELAY);
  xTimerReset(dev->ticker_timer, portMAX_DELAY);
  xTaskNotifyGive(dev->render_task_handle);
}

void reactivate_snoozed_alarms(TimerHandle_t xTimer) {
  minuteman_t *dev = (minuteman_t *)pvTimerGetTimerID(xTimer);
  minuteman_alarm_event_t ev;
  if (xSemaphoreTake(dev->mutex, 0) == pdTRUE) {
    for (int i = 0; i < 2; i++) {
      if (dev->alarms[i].snoozed) {
        ESP_LOGI(__FUNCTION__, "reactivating alarm %d after snooze", i);
        dev->alarms[i].snoozed = false;
        dev->alarms[i].active = true;
        ev.alarm_idx = i;
        ev.type = MINUTEMAN_ALARM_ACTIVE;
        xQueueSendToBack(dev->alarm_evt_queue, &ev, 0);
        break;
      }
    }
    xSemaphoreGive(dev->mutex);
  }
}

