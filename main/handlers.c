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
