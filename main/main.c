/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "hal/ledc_types.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "soc/clk_tree_defs.h"
#include "soc/gpio_num.h"
#include "wifi.h"
#include <alarm.h>
#include <button.h>
#include <encoder.h>
#include <handlers.h>
#include <minuteman.h>
#include <persistence.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>
#include <time.h>
#include "timers.h"

static button_t button_alarm0;
static button_t button_alarm1;
static button_t button_snooze;

static TimerHandle_t return_to_clock_timer;
static minuteman_t minuteman_dev;
static TaskHandle_t alarm_handler_task;
static nvs_handle_t alarm_storage_handle;

void enter_edit_mode_timers() {
  xTimerStop(minuteman_dev.ticker_timer, portMAX_DELAY);
  xTimerReset(return_to_clock_timer, portMAX_DELAY);
  xTimerReset(minuteman_dev.toggle_display_timer, portMAX_DELAY);
}

void leave_edit_mode_timers() {
  xTimerReset(minuteman_dev.ticker_timer, portMAX_DELAY);
  xTimerStop(return_to_clock_timer, portMAX_DELAY);
  xTimerStop(minuteman_dev.toggle_display_timer, portMAX_DELAY);
}

void set_timezone() {
  setenv("TZ", "CST6", 1);
  tzset();
}

void time_sync_notification_cb(struct timeval *tv) {
  ESP_LOGI(__FUNCTION__, "Notification of a time synchronization event");
}

static void initialize_sntp(void) {
  ESP_LOGI(__FUNCTION__, "Initializing SNTP");
  wifi_init_sta();
  esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
  sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
  esp_sntp_setservername(0, "pool.ntp.org");
  sntp_set_time_sync_notification_cb(time_sync_notification_cb);
  esp_sntp_init();
}

void encoder_handler(void *arg) {

  minuteman_t *dev = (minuteman_t *)arg;

  ESP_LOGI(__FUNCTION__, "encoder task started");
  rotary_encoder_event_t e;

  while (1) {
    xQueueReceive(dev->re_evt_queue, &e, portMAX_DELAY);

    switch (e.type) {
    case RE_ET_BTN_PRESSED:
      if (xSemaphoreTake(dev->mutex, 0) == pdTRUE) {
        dev->display_on = true;
        switch (dev->state) {
        case CLOCK_MODE:
          dev->state = ALARM_EDIT;
          dev->selected_alarm_idx = 0;
          enter_edit_mode_timers();
          break;
        case ALARM_EDIT:
          if (dev->selected_alarm_idx == 1) {
            dev->state = CLOCK_MODE;
            dev->display_on = true;
            leave_edit_mode_timers();
          } else {
            dev->selected_alarm_idx += 1;
            dev->state = ALARM_EDIT;
            dev->display_on = true;
            enter_edit_mode_timers();
          }
          break;
        }
        xSemaphoreGive(dev->mutex);
        ESP_LOGI(__FUNCTION__, "render called from RE button press");
        xTaskNotifyGive(dev->render_task_handle);
      }
      break;
    case RE_ET_CHANGED:
      if (xSemaphoreTake(dev->mutex, 0) == pdTRUE) {
        dev->display_on = true;
        if (dev->state == ALARM_EDIT) {
          minuteman_locked_inc_selected_alarm(dev, e.diff);
          ESP_ERROR_CHECK(
              nvs_persist_selected_alarm(alarm_storage_handle, dev));
          enter_edit_mode_timers();
        }
        xSemaphoreGive(dev->mutex);
        xTaskNotifyGive(dev->render_task_handle);
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

static void on_button0_press(button_t *btn, button_state_t state) {
  minuteman_alarm_event_t ev;
  ev.alarm_idx = 0;
  if (state == BUTTON_PRESSED) {
    ev.type = MINUTEMAN_ALARM_ENABLED;
    xQueueSendToBack(minuteman_dev.alarm_evt_queue, &ev, 0);
  }
  if (state == BUTTON_RELEASED) {
    ev.type = MINUTEMAN_ALARM_DISABLED;
    xQueueSendToBack(minuteman_dev.alarm_evt_queue, &ev, 0);
  }
}

static void on_button1_press(button_t *btn, button_state_t state) {
  minuteman_alarm_event_t ev;
  ev.alarm_idx = 1;
  if (state == BUTTON_PRESSED) {
    ev.type = MINUTEMAN_ALARM_ENABLED;
    xQueueSendToBack(minuteman_dev.alarm_evt_queue, &ev, 0);
  }
  if (state == BUTTON_RELEASED) {
    ev.type = MINUTEMAN_ALARM_DISABLED;
    xQueueSendToBack(minuteman_dev.alarm_evt_queue, &ev, 0);
  }
}

static void on_button_snooze_press(button_t *btn, button_state_t state) {
  minuteman_alarm_event_t ev = {.type = MINUTEMAN_ALARM_SNOOZED,
                                .alarm_idx = ALARM_ANY};
  if (state == BUTTON_PRESSED) {
    xQueueSendToBack(minuteman_dev.alarm_evt_queue, &ev, 0);
  }
}
esp_err_t init_alarm_buttons() {
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

  button_snooze.gpio = GPIO_NUM_27;
  button_snooze.pressed_level = 0;
  button_snooze.internal_pull = true;
  button_snooze.autorepeat = false;
  button_snooze.callback = on_button_snooze_press;
  CHECK(button_init(&button_snooze));

  return ESP_OK;
}

void app_main(void) {
  set_timezone();
  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(
      nvs_open("alarm_storage", NVS_READWRITE, &alarm_storage_handle));
  initialize_sntp();
  minuteman_init(&minuteman_dev);
  nvs_restore_stored_alarms(alarm_storage_handle, &minuteman_dev);
  init_alarm_buttons();

  minuteman_dev.ticker_timer =
      xTimerCreate("1000ms timer", pdMS_TO_TICKS(1000), pdTRUE,
                   (void *)&minuteman_dev, ticker);
  // TODO: Make flash interval and return to clock mode timeout compile time
  // configs
  return_to_clock_timer =
      xTimerCreate("return to clock mode automatically", pdMS_TO_TICKS(5000),
                   pdFALSE, (void *)&minuteman_dev, return_to_clock_mode);
  minuteman_dev.toggle_display_timer =
      xTimerCreate("toggle disple on/off", pdMS_TO_TICKS(500), pdTRUE,
                   (void *)&minuteman_dev, toggle_display);
  // TODO: Make snooze timeout a config value
  minuteman_dev.reactivate_snoozed_alarms_timer =
      xTimerCreate("reactivate snoozed alarms", pdMS_TO_TICKS(60000), pdFALSE,
                   (void *)&minuteman_dev, reactivate_snoozed_alarms);
  /* TODO: Make alarm automatic disable timeout a config value */
  minuteman_dev.alarm_disable_timer =
      xTimerCreate("disable alarm after a while", pdMS_TO_TICKS(2000 * 60),
                   pdFALSE, (void *)&minuteman_dev, disable_alarm);
  xTimerStart(minuteman_dev.ticker_timer, portMAX_DELAY);

  minuteman_dev.reenable_mask_timer = xTimerCreate(
      "reenable mask functionality after 5s", pdMS_TO_TICKS(1000 * 5), pdFALSE,
      (void *)&minuteman_dev, reenable_mask);

  xTaskCreatePinnedToCore(&alarm_handler, "alarm task",
                          configMINIMAL_STACK_SIZE * 8, (void *)&minuteman_dev,
                          5, &alarm_handler_task, 0);
  xTaskCreatePinnedToCore(&encoder_handler, "encoder task",
                          configMINIMAL_STACK_SIZE * 8, (void *)&minuteman_dev,
                          5, NULL, 0);
  xTaskCreatePinnedToCore(&render_handler, "render task",
                          configMINIMAL_STACK_SIZE * 8, (void *)&minuteman_dev,
                          5, &minuteman_dev.render_task_handle, 0);
  for (;;)
    ;
}
