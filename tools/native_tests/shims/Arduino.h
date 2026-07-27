#pragma once

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_map>

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

using std::abs;
using std::isfinite;
using std::max;
using std::min;

template <typename T>
constexpr T constrain(const T &value, const T &low, const T &high) {
  return value < low ? low : (value > high ? high : value);
}

constexpr int OUTPUT = 0x03;
constexpr int HIGH = 0x01;
constexpr int LOW = 0x00;

using TickType_t = uint32_t;
using BaseType_t = int;
using TaskFunction_t = void (*)(void *);
struct portMUX_TYPE {};

constexpr BaseType_t pdPASS = 1;
inline constexpr portMUX_TYPE portMUX_INITIALIZER_UNLOCKED = {};

namespace arduino_fake {

inline std::string serial_output;
inline uint32_t millis_value = 0;
inline uint32_t micros_value = 0;
inline uint32_t us_per_tick = 0;
inline TickType_t tick_count = 0;
inline std::unordered_map<int, uint32_t> ledc_duty_by_pin;
inline std::unordered_map<int, bool> ledc_attached_by_pin;
inline std::unordered_map<int, int> digital_level_by_pin;
inline std::unordered_map<int, int> pin_mode_by_pin;
inline bool stop_on_task_delay = false;
inline std::function<void(uint32_t)> pre_tick_hook = nullptr;
inline uint32_t tick_index = 0;
inline uint32_t tick_limit = 0;

struct TaskDelayExit {};

inline void reset() {
  serial_output.clear();
  millis_value = 0;
  micros_value = 0;
  us_per_tick = 0;
  tick_count = 0;
  ledc_duty_by_pin.clear();
  ledc_attached_by_pin.clear();
  digital_level_by_pin.clear();
  pin_mode_by_pin.clear();
  pre_tick_hook = nullptr;
  tick_index = 0;
  tick_limit = 0;
}

inline void appendFormatted(std::string &destination, const char *format,
                            va_list arguments) {
  va_list measure_arguments;
  va_copy(measure_arguments, arguments);
  const int length = std::vsnprintf(nullptr, 0, format, measure_arguments);
  va_end(measure_arguments);
  if (length <= 0) return;

  std::string formatted(static_cast<std::size_t>(length), '\0');
  std::vsnprintf(formatted.data(), formatted.size() + 1, format, arguments);
  destination += formatted;
}

}  // namespace arduino_fake

class FakeSerial {
public:
  void begin(unsigned long) {}

  template <typename T>
  void print(const T &value) {
    std::ostringstream stream;
    stream << value;
    arduino_fake::serial_output += stream.str();
  }

  void println() { arduino_fake::serial_output.push_back('\n'); }

  template <typename T>
  void println(const T &value) {
    print(value);
    println();
  }

  int printf(const char *format, ...) {
    const std::size_t before = arduino_fake::serial_output.size();
    va_list arguments;
    va_start(arguments, format);
    arduino_fake::appendFormatted(
        arduino_fake::serial_output, format, arguments);
    va_end(arguments);
    return static_cast<int>(arduino_fake::serial_output.size() - before);
  }
};

inline FakeSerial Serial;

inline uint32_t millis() { return arduino_fake::millis_value; }
inline uint32_t micros() { return arduino_fake::micros_value; }
inline void delay(uint32_t) {}
inline void delayMicroseconds(uint32_t) {}

inline void pinMode(int pin, int mode) {
  arduino_fake::pin_mode_by_pin[pin] = mode;
}

inline void digitalWrite(int pin, int level) {
  arduino_fake::digital_level_by_pin[pin] = level;
}

inline bool ledcAttach(int pin, uint32_t, uint8_t) {
  arduino_fake::ledc_attached_by_pin[pin] = true;
  return true;
}

inline bool ledcWrite(int pin, uint32_t duty) {
  arduino_fake::ledc_duty_by_pin[pin] = duty;
  return true;
}

inline TickType_t pdMS_TO_TICKS(uint32_t milliseconds) {
  return milliseconds;
}

inline TickType_t xTaskGetTickCount() {
  return arduino_fake::tick_count;
}

inline void vTaskDelayUntil(TickType_t *, TickType_t) {
  using namespace arduino_fake;
  if (pre_tick_hook) pre_tick_hook(tick_index);
  const uint64_t elapsed_us =
      static_cast<uint64_t>(micros_value % 1000U) + us_per_tick;
  micros_value += us_per_tick;
  millis_value += static_cast<uint32_t>(elapsed_us / 1000U);
  tick_index++;
  if (tick_limit && tick_index > tick_limit) throw TaskDelayExit{};
}
inline void vTaskDelay(TickType_t) {
  if (arduino_fake::stop_on_task_delay) throw arduino_fake::TaskDelayExit{};
}
inline void portENTER_CRITICAL(portMUX_TYPE *) {}
inline void portEXIT_CRITICAL(portMUX_TYPE *) {}

inline BaseType_t xTaskCreatePinnedToCore(
    TaskFunction_t, const char *, uint32_t, void *, uint32_t, void **,
    BaseType_t) {
  return pdPASS;
}
