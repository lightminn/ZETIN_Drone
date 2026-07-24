#pragma once

#include <cstdint>

class TwoWire {
public:
  void begin(int, int) {}
};

inline TwoWire Wire;
