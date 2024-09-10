#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include <minuteman.h>

esp_err_t nvs_persist_selected_alarm(nvs_handle_t handle, minuteman_t *dev) {
  switch (dev->selected_alarm_idx) {
  case 0:
    return nvs_set_u32(handle, "alarm0", (long int)dev->alarms[0].timeval);
  case 1:
    return nvs_set_u32(handle, "alarm1", (long int)dev->alarms[1].timeval);
  }
  return ESP_OK;
}

esp_err_t nvs_restore_stored_alarms(nvs_handle_t handle, minuteman_t *dev) {
  uint32_t stored;
  esp_err_t err;
  err = nvs_get_u32(handle, "alarm0", &stored);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGW(__FUNCTION__, "No stored alarm found for ALM0");
  } else if (err != ESP_OK) {
    return err;
  } else {
    dev->alarms[0].timeval = (time_t)stored;
  }
  err = nvs_get_u32(handle, "alarm1", &stored);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGW(__FUNCTION__, "No stored alarm found for ALM1");
  } else if (err != ESP_OK) {
    return err;
  } else {
    dev->alarms[1].timeval = (time_t)stored;
  }
  return ESP_OK;
}
