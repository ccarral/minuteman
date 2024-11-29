#include "esp_log.h"
#include <minuteman.h>

void render_handler(void *arg) {
  ESP_LOGI(__FUNCTION__, "render task started");
  minuteman_t *dev = (minuteman_t *)arg;
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ESP_ERROR_CHECK(minuteman_render_display(dev));
  }
}

void alarm_handler(void *arg) {
  minuteman_t *dev = (minuteman_t *)arg;
  minuteman_alarm_event_t e;
  while (1) {
    xQueueReceive(dev->alarm_evt_queue, &e, portMAX_DELAY);
    ESP_LOGI(__FUNCTION__, "alarm handler awoken");
    if (xSemaphoreTake(dev->mutex, portMAX_DELAY) == pdTRUE) {
      switch (e.type) {
      case MINUTEMAN_ALARM_ENABLED:
        minuteman_locked_set_alarm_enabled(dev, e.alarm_idx, true);
        xTaskNotifyGive(dev->render_task_handle);
        break;
      case MINUTEMAN_ALARM_ACTIVE:
        minuteman_locked_set_alarm_active(dev, e.alarm_idx);
        xTimerReset(dev->alarm_disable_timer, portMAX_DELAY);
        break;
      case MINUTEMAN_ALARM_DISABLED:
      case MINUTEMAN_ALARM_INACTIVE:
        minuteman_locked_set_alarm_enabled(dev, e.alarm_idx, false);
        minuteman_locked_set_alarm_inactive(dev, e.alarm_idx);
        xTaskNotifyGive(dev->render_task_handle);
        break;
      case MINUTEMAN_ALARM_SNOOZED:
        // If pressed, we want to reset restart the timer that will reset
        // the masked value to "enabled" after a timeout
        minuteman_locked_set_mask_disabled(dev);
        xTimerReset(dev->reenable_mask_timer, portMAX_DELAY);
        for (size_t i = 0; i < 2; i++) {
          if (dev->alarms[i].active) {
            minuteman_locked_set_snoozed_alarm(dev, i);
            xTimerReset(dev->reactivate_snoozed_alarms_timer, portMAX_DELAY);
            break;
          }
        }
        break;
      }
      // TODO: If snooze, create timer that will set alarm to active in
      // SNOZE_TIME seconds
      xSemaphoreGive(dev->mutex);
    }
  }
}
