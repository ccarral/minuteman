#include "esp_err.h"
#include "nvs.h"
#include <minuteman.h>

esp_err_t nvs_persist_selected_alarm(nvs_handle_t handle, minuteman_t *dev);

esp_err_t nvs_restore_stored_alarms(nvs_handle_t handle, minuteman_t *dev);
