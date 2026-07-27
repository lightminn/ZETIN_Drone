#pragma once

#include <Arduino.h>

// 자동착륙 상태. 텔레메트리 Failsafe_Phase로 그대로 나간다.
enum FailsafePhase : uint8_t {
  FS_NONE        = 0,
  FS_DESCENDING  = 1,
  FS_CUT_LANDED  = 2,
  FS_CUT_TIMEOUT = 3,
  FS_CUT_ABORT   = 4,
};

// 착지 감지기의 누적 상태. pid_task가 소유하고 진입 시 {}로 초기화한다.
struct LandDetector {
  bool     saw_sub_1g = false;      // 실제로 하강 가속 중이었다는 증거
  bool     settling = false;        // 1g 복귀가 진행 중인가
  uint32_t settle_start_ms = 0;
};

// 매 tick 호출. 착지가 확정되면 true를 돌려준다.
//
// 원리: 공중에서 스로틀이 호버보다 낮으면 아래로 가속하므로 |accel| < 1g다.
// 지면이 받치면 스로틀과 무관하게 1g로 돌아온다. 진입 순간에는 스로틀이 아직
// 호버라 이미 1g이므로, sub-1g를 한 번이라도 본 뒤에만 1g 복귀를 인정한다.
static inline bool updateLandDetector(
    LandDetector &det, float accel_magnitude_g, uint32_t elapsed_ms,
    float accel_tol_g, uint32_t min_descend_ms, uint32_t confirm_ms) {
  if (elapsed_ms < min_descend_ms) {
    det.settling = false;
    return false;
  }
  if (accel_magnitude_g < 1.0f - accel_tol_g) {
    det.saw_sub_1g = true;
    det.settling = false;
    return false;
  }
  if (!det.saw_sub_1g) {
    det.settling = false;
    return false;
  }
  if (fabsf(accel_magnitude_g - 1.0f) > accel_tol_g) {
    det.settling = false;      // 접지 충격 스파이크 등
    return false;
  }
  if (!det.settling) {
    det.settling = true;
    det.settle_start_ms = elapsed_ms;
    return false;
  }
  return (elapsed_ms - det.settle_start_ms) >= confirm_ms;
}

static inline int failsafeDescentThrottle(int entry_throttle,
                                          int descent_delta_us) {
  const int throttle = entry_throttle - descent_delta_us;
  return throttle < 1000 ? 1000 : throttle;
}

static inline FailsafePhase failsafeStep(bool landed, uint32_t elapsed_ms,
                                         uint32_t max_ms) {
  if (landed) return FS_CUT_LANDED;
  if (elapsed_ms >= max_ms) return FS_CUT_TIMEOUT;
  return FS_DESCENDING;
}
