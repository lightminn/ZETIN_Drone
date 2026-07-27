#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

#include "failsafe_land.h"

static int g_failures = 0;

static void runCase(const std::string &name, const std::function<void()> &body) {
  try {
    body();
    std::cout << "[PASS] " << name << '\n';
  } catch (const std::exception &error) {
    std::cerr << "[FAIL] " << name << ": " << error.what() << '\n';
    g_failures++;
  }
}

#define CHECK(expr)                                                          \
  do {                                                                       \
    if (!(expr)) throw std::runtime_error(#expr);                            \
  } while (0)

static const LandProbeConfig kProbeConfig = {
    1.0f,  // lpf_alpha
    1000,  // min_descend_ms
    400,   // period_ms
    120,   // dip_ms
    30,    // sample_delay_ms
    0.06f, // response_g
    2,     // confirm_n
    40,    // dip_us
};

static bool probeStep(LandDetector &det, float accel_magnitude_g,
                      uint32_t elapsed_ms, int base_throttle_us = 1280) {
  return updateLandDetector(
      det, accel_magnitude_g, elapsed_ms, base_throttle_us, kProbeConfig);
}

int main() {
  runCase("호버 후보 시간이 2초 누적된 뒤에만 유효해진다", [] {
    HoverThrottleEstimator estimator = {};
    for (uint32_t t = 0; t < 2000; t++) {
      updateHoverThrottleEstimator(
          estimator, true, 1340, t, 0.001f, 3.0f, 2000);
      CHECK(!estimator.valid);
    }
    updateHoverThrottleEstimator(
        estimator, true, 1340, 2000, 0.001f, 3.0f, 2000);
    CHECK(estimator.valid);
    CHECK(std::fabs(estimator.estimate_us - 1340.0f) < 1e-3f);
  });

  runCase("호버 유효시간은 부적합 구간을 제외하고 누적한다", [] {
    HoverThrottleEstimator estimator = {};
    for (uint32_t t = 0; t <= 1000; t++) {
      updateHoverThrottleEstimator(
          estimator, true, 1300, t, 0.001f, 3.0f, 2000);
    }
    for (uint32_t t = 1001; t <= 1500; t++) {
      updateHoverThrottleEstimator(
          estimator, false, 1700, t, 0.001f, 3.0f, 2000);
      CHECK(!estimator.valid);
    }
    for (uint32_t t = 1501; t < 2500; t++) {
      updateHoverThrottleEstimator(
          estimator, true, 1340, t, 0.001f, 3.0f, 2000);
      CHECK(!estimator.valid);
    }
    updateHoverThrottleEstimator(
        estimator, true, 1340, 2500, 0.001f, 3.0f, 2000);
    CHECK(estimator.valid);
    CHECK(estimator.estimate_us > 1310.0f);
    CHECK(estimator.estimate_us < 1340.0f);
  });

  runCase("유효 호버 추정치는 3초 시정수 LPF로 천천히 추적한다", [] {
    HoverThrottleEstimator estimator = {};
    for (uint32_t t = 0; t <= 2000; t++) {
      updateHoverThrottleEstimator(
          estimator, true, 1340, t, 0.001f, 3.0f, 2000);
    }
    CHECK(estimator.valid);
    for (uint32_t t = 2001; t <= 3000; t++) {
      updateHoverThrottleEstimator(
          estimator, true, 1400, t, 0.001f, 3.0f, 2000);
    }
    CHECK(estimator.estimate_us > 1356.0f);
    CHECK(estimator.estimate_us < 1358.0f);
  });

  runCase("하강 스로틀은 진입값에서 delta를 뺀 값이다", [] {
    CHECK(failsafeDescentThrottle(1360, 60) == 1300);
  });

  runCase("하강 스로틀은 1000 미만으로 내려가지 않는다", [] {
    CHECK(failsafeDescentThrottle(1020, 60) == 1000);
  });

  runCase("min_descend 전에는 필터만 갱신하고 프로브를 시작하지 않는다", [] {
    LandDetector det = {};
    CHECK(!probeStep(det, 1.20f, 0));
    CHECK(!probeStep(det, 0.80f, 500));
    CHECK(det.filt_init);
    CHECK(std::fabs(det.filt - 0.80f) < 1e-6f);
    CHECK(det.probe_state == FS_PROBE_WAIT);
    CHECK(!landDetectorProbeActive(det));
  });

  runCase("min_descend에서 딥을 시작하고 120ms 동안만 활성이다", [] {
    LandDetector det = {};
    CHECK(!probeStep(det, 1.0f, 999));
    CHECK(!probeStep(det, 1.0f, 1000));
    CHECK(det.probe_state == FS_PROBE_DIP);
    CHECK(det.probe_start_ms == 1000);
    CHECK(landDetectorProbeActive(det));
    CHECK(!probeStep(det, 1.0f, 1119));
    CHECK(landDetectorProbeActive(det));
    CHECK(!probeStep(det, 1.0f, 1120));
    CHECK(det.probe_state == FS_PROBE_EVALUATE);
    CHECK(!landDetectorProbeActive(det));
    CHECK(det.no_response_count == 1);
  });

  runCase("딥 첫 30ms의 과도는 반응 표본에서 제외한다", [] {
    LandDetector det = {};
    CHECK(!probeStep(det, 1.0f, 1000));
    CHECK(!probeStep(det, 0.70f, 1029));
    CHECK(!probeStep(det, 1.0f, 1030));
    CHECK(!probeStep(det, 1.0f, 1120));
    CHECK(std::fabs(det.last_response_g) < 1e-6f);
    CHECK(det.no_response_count == 1);
  });

  runCase("딥 중 0.06g 초과 하락은 공중 반응으로 판정한다", [] {
    LandDetector det = {};
    CHECK(!probeStep(det, 1.0f, 1000));
    CHECK(!probeStep(det, 0.88f, 1030));
    CHECK(!probeStep(det, 0.88f, 1120));
    CHECK(det.last_response_g > 0.06f);
    CHECK(det.no_response_count == 0);
  });

  runCase("연속 두 번 무반응이어야 착지를 확정한다", [] {
    LandDetector det = {};
    CHECK(!probeStep(det, 1.0f, 1000));
    CHECK(!probeStep(det, 1.0f, 1120));
    CHECK(det.no_response_count == 1);
    CHECK(!probeStep(det, 1.0f, 1121));
    CHECK(det.probe_state == FS_PROBE_WAIT);
    CHECK(!probeStep(det, 1.0f, 1400));
    CHECK(probeStep(det, 1.0f, 1520));
    CHECK(det.no_response_count == 2);
  });

  runCase("두 무반응 사이의 공중 반응은 연속 카운트를 지운다", [] {
    LandDetector det = {};
    CHECK(!probeStep(det, 1.0f, 1000));
    CHECK(!probeStep(det, 1.0f, 1120));
    CHECK(det.no_response_count == 1);
    CHECK(!probeStep(det, 1.0f, 1121));
    CHECK(!probeStep(det, 1.0f, 1400));
    CHECK(!probeStep(det, 0.80f, 1430));
    CHECK(!probeStep(det, 0.80f, 1520));
    CHECK(det.no_response_count == 0);
    CHECK(!probeStep(det, 1.0f, 1521));
    CHECK(!probeStep(det, 1.0f, 1800));
    CHECK(!probeStep(det, 1.0f, 1920));
    CHECK(det.no_response_count == 1);
  });

  runCase("판정은 절대 1g가 아니라 딥 직전 대비 차분이다", [] {
    LandDetector det = {};
    CHECK(!probeStep(det, 0.90f, 1000));
    CHECK(!probeStep(det, 0.90f, 1030));
    CHECK(!probeStep(det, 0.90f, 1120));
    CHECK(std::fabs(det.last_response_g) < 1e-6f);
    CHECK(det.no_response_count == 1);
  });

  runCase("딥이 1000us 아래로 가면 프로브 불가로 남고 착지하지 않는다", [] {
    LandDetector det = {};
    for (uint32_t t = 0; t <= 5000; t += 10) {
      CHECK(!probeStep(det, 1.0f, t, 1030));
    }
    CHECK(det.probe_state == FS_PROBE_UNAVAILABLE);
    CHECK(!landDetectorProbeActive(det));
    CHECK(det.no_response_count == 0);
  });

  runCase("LPF는 첫 샘플을 대입하고 이후 alpha로 갱신한다", [] {
    LandDetector det = {};
    const LandProbeConfig config = {
        0.25f, 1000, 400, 120, 30, 0.06f, 2, 40};
    CHECK(!updateLandDetector(det, 1.20f, 0, 1280, config));
    CHECK(det.filt_init);
    CHECK(std::fabs(det.filt - 1.20f) < 1e-6f);
    CHECK(!updateLandDetector(det, 0.80f, 10, 1280, config));
    CHECK(std::fabs(det.filt - 1.10f) < 1e-6f);
  });

  runCase("failsafeStep: 착지가 timeout보다 우선한다", [] {
    CHECK(failsafeStep(true, 9999, 5000) == FS_CUT_LANDED);
  });

  runCase("failsafeStep: 백스톱 시간이 지나면 종료한다", [] {
    CHECK(failsafeStep(false, 5000, 5000) == FS_CUT_TIMEOUT);
    CHECK(failsafeStep(false, 4999, 5000) == FS_DESCENDING);
  });

  if (g_failures != 0) {
    std::cerr << g_failures << " failsafe-land case(s) failed\n";
    return 1;
  }
  std::cout << "16/16 failsafe-land cases passed\n";
  return 0;
}
