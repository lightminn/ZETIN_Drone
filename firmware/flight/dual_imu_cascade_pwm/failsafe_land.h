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
  float ground_accel_tol_g;
  uint8_t confirm_n;
  int dip_us;
};

// hover_est에서 1000us를 뺀 유효 collective에 비례해 딥을 만든다. 측정
// 가능한 하한과 제어 authority 상한은 런타임 값이므로 적용값과 clamp 여부를
// 함께 반환한다. upper_exclusive_us는 반드시 min_us보다 커야 한다.
static inline int failsafeProbeDipUs(
    float hover_estimate_us, float dip_frac, int min_us,
    int upper_exclusive_us, bool &clamped) {
  if (!isfinite(hover_estimate_us) || !isfinite(dip_frac) ||
      dip_frac <= 0.0f || upper_exclusive_us <= min_us) {
    clamped = true;
    return min_us;
  }

  const int requested_us = (int)lroundf(
      (hover_estimate_us - 1000.0f) * dip_frac);
  const int applied_us =
      requested_us < min_us
          ? min_us
          : (requested_us >= upper_exclusive_us
                 ? upper_exclusive_us - 1
                 : requested_us);
  clamped = applied_us != requested_us;
  return applied_us;
}

// resume 거부 사유. .ino 안에 두면 arduino-cli가 자동 생성한 함수
// 프로토타입이 enum 선언보다 앞에 삽입돼 컴파일이 깨진다(헤더는 그 전처리
// 대상이 아니다). 네이티브 테스트는 .ino를 g++로 직접 컴파일해 이 단계가
// 없으므로 이 결함을 잡지 못한다 — 아두이노 컴파일이 유일한 관문이다.
enum ResumeRefusalReason : uint8_t {
  RESUME_ALLOWED = 0,
  RESUME_REFUSED_PHASE,
  RESUME_REFUSED_RC,
  RESUME_REFUSED_TILT,
  RESUME_REFUSED_IMU,
  RESUME_REFUSED_HOVER,
  RESUME_REFUSED_CUMULATIVE,
};

static inline const char *resumeRefusalName(ResumeRefusalReason reason) {
  switch (reason) {
    case RESUME_REFUSED_PHASE: return "phase";
    case RESUME_REFUSED_RC: return "rc";
    case RESUME_REFUSED_TILT: return "tilt";
    case RESUME_REFUSED_IMU: return "imu";
    case RESUME_REFUSED_HOVER: return "hover";
    case RESUME_REFUSED_CUMULATIVE: return "cumulative";
    default: return "unknown";
  }
}

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
  // 과거의 폐기된 방식은 1g 복귀를 착지의 충분조건으로 썼다. 여기서는 능동
  // 프로브 연속 무반응이 여전히 주 판별이며, 지면에 정지하면 반드시 1g라는
  // 사실만 이용해 1g 근처를 착지 확정의 필요조건으로 추가한다. 1g만으로는
  // 절대로 착지를 선언하지 않는다.
  const bool near_stationary_1g =
      fabsf(det.filt - 1.0f) <= config.ground_accel_tol_g;
  return det.no_response_count >= config.confirm_n && near_stationary_1g;
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
