#pragma once

#include <cmath>

// Hardware-independent motor allocation result. Motor order is FL, RR, FR, RL.
struct ControlAllocation {
  int motor[4];
  float collective_us;
  float rp_scale;
  float yaw_scale;
  bool scaled;
};

static inline int controlAllocatorClampInt(int value, int low, int high) {
  return value < low ? low : (value > high ? high : value);
}

static inline float controlAllocatorFinite(float value) {
  return std::isfinite(value) ? value : 0.0f;
}

static inline ControlAllocation allocateControl(float roll, float pitch, float yaw,
                                                 int throttle, int minMotor,
                                                 int maxMotor) {
  ControlAllocation out = {};
  minMotor = controlAllocatorClampInt(minMotor, 1000, 2000);
  maxMotor = controlAllocatorClampInt(maxMotor, minMotor, 2000);

  const double rollValue = controlAllocatorFinite(roll);
  const double pitchValue = controlAllocatorFinite(pitch);
  const double yawValue = controlAllocatorFinite(yaw);
  const double available = static_cast<double>(maxMotor - minMotor);

  double rp[4] = {
      -pitchValue + rollValue,
      pitchValue - rollValue,
      -pitchValue - rollValue,
      pitchValue + rollValue,
  };
  const double yawVector[4] = {
      -yawValue, -yawValue, yawValue, yawValue,
  };

  double rpMin = rp[0];
  double rpMax = rp[0];
  for (int index = 1; index < 4; ++index) {
    if (rp[index] < rpMin) rpMin = rp[index];
    if (rp[index] > rpMax) rpMax = rp[index];
  }

  const double rpSpan = rpMax - rpMin;
  double rpScale = 1.0;
  if (rpSpan > available && rpSpan > 0.0) rpScale = available / rpSpan;
  for (int index = 0; index < 4; ++index) rp[index] *= rpScale;

  // λ is the largest yaw fraction for which every motor-pair span remains
  // inside the residual range after roll/pitch has claimed its priority.
  double yawScale = 1.0;
  for (int first = 0; first < 4; ++first) {
    for (int second = first + 1; second < 4; ++second) {
      const double rpDifference = rp[first] - rp[second];
      const double yawDifference = yawVector[first] - yawVector[second];
      if (yawDifference == 0.0) continue;

      const double boundA = (-available - rpDifference) / yawDifference;
      const double boundB = (available - rpDifference) / yawDifference;
      const double upper = boundA > boundB ? boundA : boundB;
      if (upper < yawScale) yawScale = upper;
    }
  }
  if (yawScale < 0.0) yawScale = 0.0;
  if (yawScale > 1.0) yawScale = 1.0;

  double diff[4] = {};
  double minDiff = 0.0;
  double maxDiff = 0.0;
  for (int index = 0; index < 4; ++index) {
    diff[index] = rp[index] + yawScale * yawVector[index];
    if (index == 0 || diff[index] < minDiff) minDiff = diff[index];
    if (index == 0 || diff[index] > maxDiff) maxDiff = diff[index];
  }

  const double collectiveLo = static_cast<double>(minMotor) - minDiff;
  const double collectiveHi = static_cast<double>(maxMotor) - maxDiff;
  double collective = static_cast<double>(throttle);
  if (collective < collectiveLo) collective = collectiveLo;
  if (collective > collectiveHi) collective = collectiveHi;

  out.collective_us = static_cast<float>(collective);
  out.rp_scale = static_cast<float>(rpScale);
  out.yaw_scale = static_cast<float>(yawScale);
  out.scaled = rpScale < 1.0 || yawScale < 1.0;
  for (int index = 0; index < 4; ++index) {
    const int rounded = static_cast<int>(std::lround(collective + diff[index]));
    out.motor[index] = controlAllocatorClampInt(rounded, minMotor, maxMotor);
  }
  return out;
}
