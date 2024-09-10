#ifndef ALARM_H
#define ALARM_H

#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include <max7219.h>
#include <minuteman.h>
#include <time.h>

#define CHECK(x)                                                               \
  do {                                                                         \
    esp_err_t __;                                                              \
    if ((__ = x) != ESP_OK)                                                    \
      return __;                                                               \
  } while (0)

#endif
