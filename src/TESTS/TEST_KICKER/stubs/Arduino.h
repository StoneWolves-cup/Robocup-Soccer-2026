#pragma once
#include <stddef.h>
#include <stdint.h>

constexpr int HIGH = 1;
constexpr int LOW = 0;
constexpr int OUTPUT = 1;
constexpr int INPUT_PULLDOWN = 2;
using portMUX_TYPE = int;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(lock) ((void)(lock))
#define portEXIT_CRITICAL(lock) ((void)(lock))

void pinMode(int pin, int mode);
void digitalWrite(int pin, int value);
int digitalRead(int pin);
uint32_t millis();
struct SerialStub {
  template <typename... Args> void printf(const char *, Args...) {}
  void println(const char *) {}
};
extern SerialStub Serial;
