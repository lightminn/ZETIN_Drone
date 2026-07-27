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

// 자동착륙 능동 프로브 상태. 그대로 텔레메트리에 내보내므로 값은 append-only
// wire contract다.
enum FailsafeProbeState : uint8_t {
  FS_PROBE_WAIT        = 0,
  FS_PROBE_DIP         = 1,
  FS_PROBE_EVALUATE    = 2,
  FS_PROBE_UNAVAILABLE = 3,
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

struct LandProbeConfig {
  float lpf_alpha;
  uint32_t min_descend_ms;
  uint32_t period_ms;
  uint32_t dip_ms;
  uint32_t sample_delay_ms;
  float response_g;
  uint8_t confirm_n;
  int dip_us;
};

// 착지 감지기의 누적 상태. pid_task가 소유하고 진입 시 {}로 초기화한다.
struct LandDetector {
  bool     filt_init = false;
  float    filt = 1.0f;             // LPF된 |accel| (g)
  FailsafeProbeState probe_state = FS_PROBE_WAIT;
  uint32_t probe_start_ms = 0;
  float    pre_dip_g = 1.0f;
  float    dip_min_g = 1.0f;
  float    last_response_g = 0.0f;  // pre_dip_g - dip_min_g
  uint8_t  no_response_count = 0;
};

static inline bool landDetectorProbeActive(const LandDetector &det) {
  return det.probe_state == FS_PROBE_DIP;
}

// 매 tick 호출. 검출기가 대기→딥→판정을 전부 소유하고 착지 확정 시 true다.
// pid_task는 반환값과 landDetectorProbeActive()만 제어에 사용한다.
//
// LPF는 min_descend 게이트보다 먼저 항상 갱신한다. 딥 직전값과 딥 중 최솟값의
// 차분을 쓰므로 5Hz 아래 저주파 진동은 400ms 프로브 안에서 공통모드로
// 상쇄된다. 반응(response_g 초과)은 공중, 연속 confirm_n회 무반응은 지면이다.
static inline bool updateLandDetector(
    LandDetector &det, float accel_magnitude_g, uint32_t elapsed_ms,
    int base_throttle_us, const LandProbeConfig &config) {
  if (!det.filt_init) {
    det.filt = accel_magnitude_g;
    det.filt_init = true;
  } else {
    det.filt += config.lpf_alpha * (accel_magnitude_g - det.filt);
  }

  if (det.probe_state == FS_PROBE_UNAVAILABLE) {
    return false;
  }

  if (elapsed_ms < config.min_descend_ms) {
    return false;
  }

  // 판정과 다음 대기 전환을 별도 tick으로 유지한다. 20Hz 텔레메트리는
  // 지속되는 DIP/UNAVAILABLE 상태와 누적 카운터/마지막 응답을 주 진단으로 쓴다.
  if (det.probe_state == FS_PROBE_EVALUATE) {
    det.probe_state = FS_PROBE_WAIT;
    return false;
  }

  if (det.probe_state == FS_PROBE_WAIT) {
    if (base_throttle_us - config.dip_us < 1000) {
      det.probe_state = FS_PROBE_UNAVAILABLE;
      return false;
    }
    const uint32_t next_probe_ms =
        det.probe_start_ms == 0
            ? config.min_descend_ms
            : det.probe_start_ms + config.period_ms;
    if (elapsed_ms < next_probe_ms) return false;

    det.probe_state = FS_PROBE_DIP;
    det.probe_start_ms = elapsed_ms;
    det.pre_dip_g = det.filt;
    det.dip_min_g = det.filt;
    return false;
  }

  const uint32_t dip_elapsed_ms = elapsed_ms - det.probe_start_ms;
  if (dip_elapsed_ms >= config.sample_delay_ms &&
      det.filt < det.dip_min_g) {
    det.dip_min_g = det.filt;
  }
  if (dip_elapsed_ms < config.dip_ms) {
    return false;
  }

  det.last_response_g = det.pre_dip_g - det.dip_min_g;
  det.probe_state = FS_PROBE_EVALUATE;
  if (det.last_response_g > config.response_g) {
    det.no_response_count = 0;
  } else if (det.no_response_count < UINT8_MAX) {
    det.no_response_count++;
  }
  return det.no_response_count >= config.confirm_n;
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
