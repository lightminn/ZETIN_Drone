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

// 호버 후보인 시간만 누적한다. 부적합 구간은 누적과 LPF를 멈추지만 이미
// 모은 시간/추정치는 보존한다. 일단 유효해진 추정치도 비행 중 후보 조건이
// 잠시 깨져도 보존하고, 유효한 샘플에서만 느린 LPF로 갱신한다.
struct HoverThrottleEstimator {
  float estimate_us = 0.0f;
  bool initialized = false;
  bool timing_initialized = false;
  bool valid = false;
  uint32_t last_update_ms = 0;
  uint32_t eligible_ms = 0;
};

static inline void updateHoverThrottleEstimator(
    HoverThrottleEstimator &estimator, bool sample_eligible,
    int throttle_us, uint32_t now_ms, float dt_s, float tau_s,
    uint32_t valid_ms) {
  uint32_t elapsed_ms = 0;
  if (estimator.timing_initialized) {
    elapsed_ms = now_ms - estimator.last_update_ms;
  } else {
    estimator.timing_initialized = true;
  }
  estimator.last_update_ms = now_ms;

  if (!sample_eligible) return;

  if (!estimator.initialized) {
    estimator.estimate_us = (float)throttle_us;
    estimator.initialized = true;
  } else {
    const float alpha = dt_s / (tau_s + dt_s);
    estimator.estimate_us +=
        alpha * ((float)throttle_us - estimator.estimate_us);
  }

  const uint32_t room = UINT32_MAX - estimator.eligible_ms;
  estimator.eligible_ms += elapsed_ms < room ? elapsed_ms : room;
  if (estimator.eligible_ms >= valid_ms) {
    estimator.valid = true;
  }
}

// 착지 감지기의 누적 상태. pid_task가 소유하고 진입 시 {}로 초기화한다.
struct LandDetector {
  bool     filt_init = false;
  float    filt = 1.0f;             // LPF된 |accel| (g)
  bool     saw_sub_1g = false;      // 실제로 하강 가속했다는 증거
  bool     saw_impact = false;      // 지면 반력 스파이크
  bool     settling = false;
  uint32_t settle_start_ms = 0;
};

// 매 tick 호출. 착지가 확정되면 true를 돌려준다.
//
// 원리: 하강 과도의 sub-1g와 지면 반력의 1g 초과 스파이크를 모두 본 뒤,
// LPF된 |accel|이 1g 근처에서 안정될 때만 착지를 인정한다. 증거 수집은
// min_descend 판정 게이트보다 항상 먼저 실행해 진입 직후 증거를 버리지 않는다.
static inline bool updateLandDetector(
    LandDetector &det, float accel_magnitude_g, uint32_t elapsed_ms,
    float lpf_alpha, float settle_tol_g, float impact_g,
    uint32_t min_descend_ms, uint32_t confirm_ms) {
  if (!det.filt_init) {
    det.filt = accel_magnitude_g;
    det.filt_init = true;
  } else {
    det.filt += lpf_alpha * (accel_magnitude_g - det.filt);
  }

  if (det.filt < 1.0f - settle_tol_g) {
    det.saw_sub_1g = true;
    det.settling = false;
    return false;
  }
  if (det.filt > 1.0f + impact_g) {
    det.saw_impact = true;
    det.settling = false;
    return false;
  }

  if (elapsed_ms < min_descend_ms) {
    det.settling = false;
    return false;
  }
  if (!det.saw_sub_1g || !det.saw_impact) {
    det.settling = false;
    return false;
  }
  if (fabsf(det.filt - 1.0f) > settle_tol_g) {
    det.settling = false;
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
