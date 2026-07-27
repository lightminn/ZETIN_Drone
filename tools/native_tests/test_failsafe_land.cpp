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

static const float kAlpha = 1.0f;
static const float kSettleTol = 0.10f;
static const float kImpact = 0.25f;
static const uint32_t kMinDescend = 1000;
static const uint32_t kConfirm = 400;

int main() {
  runCase("하강 스로틀은 진입값에서 delta를 뺀 값이다", [] {
    CHECK(failsafeDescentThrottle(1360, 60) == 1300);
  });

  runCase("하강 스로틀은 1000 미만으로 내려가지 않는다", [] {
    CHECK(failsafeDescentThrottle(1020, 60) == 1000);
  });

  runCase("회귀: 진입 직후 1g여도 착지로 판정하지 않는다", [] {
    // 진입 순간 스로틀이 아직 호버라 |accel|=1g다. 여기서 착지로 보면
    // 공중에서 모터가 꺼진다.
    LandDetector det = {};
    for (uint32_t t = 0; t < kMinDescend; t += 10) {
      CHECK(!updateLandDetector(
          det, 1.0f, t, kAlpha, kSettleTol, kImpact,
          kMinDescend, kConfirm));
    }
  });

  runCase("sub-1g를 못 본 채 1g가 계속되면 착지가 아니다", [] {
    // min_descend 시간이 지나도 하강 증거가 없으면 인정하지 않는다.
    LandDetector det = {};
    for (uint32_t t = 0; t < 4000; t += 10) {
      CHECK(!updateLandDetector(
          det, 1.0f, t, kAlpha, kSettleTol, kImpact,
          kMinDescend, kConfirm));
    }
    CHECK(!det.saw_sub_1g);
  });

  runCase("sub-1g와 충격 뒤 1g가 confirm 시간 유지되면 착지다", [] {
    LandDetector det = {};
    CHECK(!updateLandDetector(
        det, 0.80f, 1200, kAlpha, kSettleTol, kImpact,
        kMinDescend, kConfirm));
    CHECK(det.saw_sub_1g);
    CHECK(!updateLandDetector(
        det, 1.30f, 1300, kAlpha, kSettleTol, kImpact,
        kMinDescend, kConfirm));
    CHECK(det.saw_impact);
    CHECK(!updateLandDetector(
        det, 1.00f, 1500, kAlpha, kSettleTol, kImpact,
        kMinDescend, kConfirm));
    CHECK(!updateLandDetector(
        det, 1.00f, 1800, kAlpha, kSettleTol, kImpact,
        kMinDescend, kConfirm));
    CHECK(updateLandDetector(
        det, 1.00f, 1900, kAlpha, kSettleTol, kImpact,
        kMinDescend, kConfirm));
  });

  runCase("충격 없이 등속 하강의 1g로 돌아오면 착지가 아니다", [] {
    LandDetector det = {};
    CHECK(!updateLandDetector(
        det, 0.80f, 200, kAlpha, kSettleTol, kImpact,
        kMinDescend, kConfirm));
    for (uint32_t t = kMinDescend; t < 4000; t += 10) {
      CHECK(!updateLandDetector(
          det, 1.0f, t, kAlpha, kSettleTol, kImpact,
          kMinDescend, kConfirm));
    }
    CHECK(det.saw_sub_1g);
    CHECK(!det.saw_impact);
    CHECK(!det.settling);
  });

  runCase("confirm 도중 다시 sub-1g면 확정이 취소된다", [] {
    LandDetector det = {};
    updateLandDetector(
        det, 0.80f, 1200, kAlpha, kSettleTol, kImpact,
        kMinDescend, kConfirm);
    updateLandDetector(
        det, 1.30f, 1300, kAlpha, kSettleTol, kImpact,
        kMinDescend, kConfirm);
    CHECK(!updateLandDetector(
        det, 1.00f, 1500, kAlpha, kSettleTol, kImpact,
        kMinDescend, kConfirm));
    CHECK(!updateLandDetector(
        det, 0.80f, 1600, kAlpha, kSettleTol, kImpact,
        kMinDescend, kConfirm));
    CHECK(!det.settling);
    // 다시 처음부터 confirm을 채워야 한다
    CHECK(!updateLandDetector(
        det, 1.00f, 1700, kAlpha, kSettleTol, kImpact,
        kMinDescend, kConfirm));
    CHECK(!updateLandDetector(
        det, 1.00f, 2000, kAlpha, kSettleTol, kImpact,
        kMinDescend, kConfirm));
    CHECK(updateLandDetector(
        det, 1.00f, 2100, kAlpha, kSettleTol, kImpact,
        kMinDescend, kConfirm));
  });

  runCase("충격 스파이크는 증거로 남기고 즉시 착지로 보지 않는다", [] {
    LandDetector det = {};
    updateLandDetector(
        det, 0.80f, 1200, kAlpha, kSettleTol, kImpact,
        kMinDescend, kConfirm);
    CHECK(!updateLandDetector(
        det, 1.50f, 1500, kAlpha, kSettleTol, kImpact,
        kMinDescend, kConfirm));
    CHECK(det.saw_impact);
    CHECK(!det.settling);
  });

  runCase("min_descend 이전의 sub-1g도 증거로 기억한다", [] {
    LandDetector det = {};
    CHECK(!updateLandDetector(
        det, 0.80f, 500, kAlpha, kSettleTol, kImpact,
        kMinDescend, kConfirm));
    CHECK(det.saw_sub_1g);
  });

  runCase("min_descend 이전의 충격도 증거로 기억한다", [] {
    LandDetector det = {};
    CHECK(!updateLandDetector(
        det, 1.30f, 500, kAlpha, kSettleTol, kImpact,
        kMinDescend, kConfirm));
    CHECK(det.saw_impact);
  });

  runCase("LPF는 첫 샘플을 대입하고 이후 alpha로 갱신한다", [] {
    LandDetector det = {};
    CHECK(!updateLandDetector(
        det, 1.20f, 0, 0.25f, kSettleTol, kImpact,
        kMinDescend, kConfirm));
    CHECK(det.filt_init);
    CHECK(std::fabs(det.filt - 1.20f) < 1e-6f);
    CHECK(!updateLandDetector(
        det, 0.80f, 10, 0.25f, kSettleTol, kImpact,
        kMinDescend, kConfirm));
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
  std::cout << "13/13 failsafe-land cases passed\n";
  return 0;
}
