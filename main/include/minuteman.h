#ifndef MINUTEMAN_H

#define MINUTEMAN_H

#include "freertos/FreeRTOS.h"
#include "button.h"
#include "encoder.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include <alarm.h>
#include <max7219.h>
#include <time.h>

#define CHECK(x)                                                               \
  do {                                                                         \
    esp_err_t __;                                                              \
    if ((__ = x) != ESP_OK)                                                    \
      return __;                                                               \
  } while (0)

#define ALARM_0 0
#define ALARM_1 1
#define ALARM_ANY 0xFFFFFFFF

typedef struct {
  bool enabled;
  bool active;
  bool snoozed;
  time_t timeval;
  button_t button;
} minuteman_alarm_t;

enum minuteman_alarm_event_type {
  MINUTEMAN_ALARM_ENABLED,
  MINUTEMAN_ALARM_DISABLED,
  MINUTEMAN_ALARM_ACTIVE,
  MINUTEMAN_ALARM_INACTIVE,
  MINUTEMAN_ALARM_SNOOZED,
};

typedef struct {
  enum minuteman_alarm_event_type type;
  size_t alarm_idx;
} minuteman_alarm_event_t;

// TODO: Set in config
#define ENCODER_INPUT_SEC_MULTIPLIER (60 * 5)

typedef enum { CLOCK_MODE, ALARM_EDIT } minuteman_state_t;

typedef struct {
  char digits[9];
  minuteman_state_t state;
  max7219_t *display;
  SemaphoreHandle_t mutex;
  time_t current_time;
  minuteman_alarm_t alarms[2];
  int selected_alarm_idx;
  bool display_on;
  // When mask is enabled, the render function will check
  // the mask table and make a decision on the display "on-ness"
  // add special functionality to the snooze button to momentarily re-enable
  // brightness to its current time-appropiate value
  bool mask_enabled;
  struct tm timeinfo;
  QueueHandle_t alarm_evt_queue;
  QueueHandle_t re_evt_queue;
  rotary_encoder_t encoder;

  TaskHandle_t render_task_handle;

  TimerHandle_t ticker_timer;
  TimerHandle_t toggle_display_timer;
  TimerHandle_t alarm_disable_timer;
  TimerHandle_t reactivate_snoozed_alarms_timer;
  TimerHandle_t reenable_mask_timer;
} minuteman_t;

esp_err_t minuteman_render_display(minuteman_t *dev);

esp_err_t minuteman_init(minuteman_t *dev);

void minuteman_locked_inc_selected_alarm(minuteman_t *dev, int32_t diff);

void minuteman_locked_set_alarm_enabled(minuteman_t *dev, size_t alarm_idx,
                                        bool enabled);
void minuteman_locked_set_alarm_active(minuteman_t *dev, size_t alarm_idx);
void minuteman_locked_set_alarm_inactive(minuteman_t *dev, size_t alarm_idx);
void minuteman_locked_set_snoozed_alarm(minuteman_t *dev, size_t alarm_idx);

bool minuteman_alarm_check_active(minuteman_t *dev, int alarm_idx);

void minuteman_locked_stop_alarm_sound(minuteman_t *dev);

void minuteman_locked_sound_alarm(minuteman_t *dev);

void minuteman_locked_set_mask_disabled(minuteman_t *dev);

#endif /* ifndef  MINUTEMAN_H */
