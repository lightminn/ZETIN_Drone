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

static const float kTol = 0.05f;
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
      CHECK(!updateLandDetector(det, 1.0f, t, kTol, kMinDescend, kConfirm));
    }
  });

  runCase("sub-1g를 못 본 채 1g가 계속되면 착지가 아니다", [] {
    // min_descend 시간이 지나도 하강 증거가 없으면 인정하지 않는다.
    LandDetector det = {};
    for (uint32_t t = 0; t < 4000; t += 10) {
      CHECK(!updateLandDetector(det, 1.0f, t, kTol, kMinDescend, kConfirm));
    }
    CHECK(!det.saw_sub_1g);
  });

  runCase("sub-1g 뒤 1g가 confirm 시간 유지되면 착지다", [] {
    LandDetector det = {};
    CHECK(!updateLandDetector(det, 0.80f, 1200, kTol, kMinDescend, kConfirm));
    CHECK(det.saw_sub_1g);
    CHECK(!updateLandDetector(det, 1.00f, 1500, kTol, kMinDescend, kConfirm));
    CHECK(!updateLandDetector(det, 1.00f, 1800, kTol, kMinDescend, kConfirm));
    CHECK(updateLandDetector(det, 1.00f, 1900, kTol, kMinDescend, kConfirm));
  });

  runCase("confirm 도중 다시 sub-1g면 확정이 취소된다", [] {
    LandDetector det = {};
    updateLandDetector(det, 0.80f, 1200, kTol, kMinDescend, kConfirm);
    CHECK(!updateLandDetector(det, 1.00f, 1500, kTol, kMinDescend, kConfirm));
    CHECK(!updateLandDetector(det, 0.80f, 1600, kTol, kMinDescend, kConfirm));
    CHECK(!det.settling);
    // 다시 처음부터 confirm을 채워야 한다
    CHECK(!updateLandDetector(det, 1.00f, 1700, kTol, kMinDescend, kConfirm));
    CHECK(!updateLandDetector(det, 1.00f, 2000, kTol, kMinDescend, kConfirm));
    CHECK(updateLandDetector(det, 1.00f, 2100, kTol, kMinDescend, kConfirm));
  });

  runCase("허용오차 밖의 1g 초과도 착지로 보지 않는다", [] {
    LandDetector det = {};
    updateLandDetector(det, 0.80f, 1200, kTol, kMinDescend, kConfirm);
    // 접지 충격 스파이크(1.5g)는 지면 지지 상태가 아니다
    CHECK(!updateLandDetector(det, 1.50f, 1500, kTol, kMinDescend, kConfirm));
    CHECK(!det.settling);
  });

  runCase("min_descend 이전의 sub-1g는 기억하지 않는다", [] {
    LandDetector det = {};
    CHECK(!updateLandDetector(det, 0.80f, 500, kTol, kMinDescend, kConfirm));
    CHECK(!det.saw_sub_1g);
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
  std::cout << "10/10 failsafe-land cases passed\n";
  return 0;
}
