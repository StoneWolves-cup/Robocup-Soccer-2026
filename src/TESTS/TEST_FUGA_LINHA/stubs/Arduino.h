#ifndef TEST_ARDUINO_H
#define TEST_ARDUINO_H

struct TestSerial {
  void println(const char *) {}
};

extern TestSerial Serial;

#endif
