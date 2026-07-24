#pragma once

#include <cstdint>

#include "Wire.h"

enum eBmm350PowerModes_t {
  eBmm350NormalMode = 1,
};

constexpr uint8_t BMM350_DATA_RATE_50HZ = 50;

struct sBmm350MagData_t {
  int32_t x = 0;
  int32_t y = 0;
  int32_t z = 0;
  int32_t temperature = 0;
  float float_x = 0.0f;
  float float_y = 0.0f;
  float float_z = 0.0f;
  float float_temperature = 0.0f;
};

class DFRobot_BMM350_I2C {
public:
  DFRobot_BMM350_I2C(TwoWire *, uint8_t) {}

  uint8_t begin() { return begin_result; }
  void setOperationMode(eBmm350PowerModes_t) {}
  void setRate(uint8_t) {}
  sBmm350MagData_t getGeomagneticData() { return next_data; }

  uint8_t begin_result = 0;
  sBmm350MagData_t next_data;
};
