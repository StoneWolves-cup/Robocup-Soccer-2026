#pragma once
#include <stdint.h>
using esp_err_t = int;
constexpr esp_err_t ESP_OK = 0;
using esp_timer_handle_t = void *;
struct esp_timer_create_args_t {
  void (*callback)(void *);
  void *arg;
  const char *name;
};
esp_err_t esp_timer_create(const esp_timer_create_args_t *, esp_timer_handle_t *);
esp_err_t esp_timer_start_once(esp_timer_handle_t, uint64_t);
const char *esp_err_to_name(esp_err_t);
