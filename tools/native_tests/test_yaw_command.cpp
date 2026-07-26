#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

#include "yaw_command.h"

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

static void checkNear(float actual, float expected, float tol,
                      const char *what) {
  if (std::fabs(actual - expected) > tol) {
    throw std::runtime_error(std::string(what) + ": got " +
                             std::to_string(actual) + " want " +
                             std::to_string(expected));
  }
}

static const float kDead = 3.0f;
static const float kSettle = 10.0f;

int main() {
  runCase("스틱 편향 중에는 rate 모드이고 setpoint가 현재 heading을 따라간다", [] {
    YawOuter s = updateYawOuter(45.0f, 40.0f, 137.0f, 0.0f, false, kDead, kSettle);
    CHECK(!s.hold);
    checkNear(s.target_angle_deg, 137.0f, 1e-4f, "슬레이빙");
    checkNear(yawTargetRateDps(s, 45.0f, 137.0f, 3.0f, 180.0f), 45.0f, 1e-4f,
              "rate 명령 통과");
  });

  runCase("스틱을 놓아도 아직 회전 중이면 잠기지 않는다", [] {
    YawOuter s = updateYawOuter(0.0f, 40.0f, 137.0f, 0.0f, false, kDead, kSettle);
    CHECK(!s.hold);
    checkNear(s.target_angle_deg, 137.0f, 1e-4f, "계속 슬레이빙");
    checkNear(yawTargetRateDps(s, 0.0f, 137.0f, 3.0f, 180.0f), 0.0f, 1e-4f,
              "감쇠 목표 0");
  });

  runCase("스틱 중립 + 정착이면 그 자리에서 잠긴다", [] {
    YawOuter s = updateYawOuter(0.0f, 2.0f, 137.0f, 137.0f, false, kDead, kSettle);
    CHECK(s.hold);
    checkNear(s.target_angle_deg, 137.0f, 1e-4f, "setpoint 동결");
    checkNear(yawTargetRateDps(s, 0.0f, 137.0f, 3.0f, 180.0f), 0.0f, 1e-4f,
              "오차 0이면 명령 0");
  });

  runCase("잠긴 뒤 heading이 밀리면 복원 방향으로 명령한다", [] {
    YawOuter s = updateYawOuter(0.0f, 1.0f, 130.0f, 137.0f, false, kDead, kSettle);
    CHECK(s.hold);
    // 오차 = wrapDeg(137 - 130) = +7 → +7 * 3.0 = +21
    checkNear(yawTargetRateDps(s, 0.0f, 130.0f, 3.0f, 180.0f), 21.0f, 1e-3f,
              "복원 명령");
  });

  runCase("회귀: 큰 heading 드리프트 상태로 진입해도 슬램이 없다", [] {
    // 문제 3 재현 조건. 슬레이빙이 매 tick 돌았으므로 prev setpoint는 현재값이다.
    YawOuter s = updateYawOuter(0.0f, 1.0f, 150.0f, 150.0f, false, kDead, kSettle);
    CHECK(s.hold);
    checkNear(yawTargetRateDps(s, 0.0f, 150.0f, 3.0f, 180.0f), 0.0f, 1e-4f,
              "슬램 없음");
  });

  runCase("오버라이드는 스틱·각속도와 무관하게 잠금을 강제한다", [] {
    YawOuter s = updateYawOuter(45.0f, 40.0f, 130.0f, 0.0f, true, kDead, kSettle);
    CHECK(s.hold);
    checkNear(s.target_angle_deg, 0.0f, 1e-4f, "setpoint 동결 유지");
    // 오차 = wrapDeg(0 - 130) = -130 → -390 → -180으로 클램프
    checkNear(yawTargetRateDps(s, 45.0f, 130.0f, 3.0f, 180.0f), -180.0f, 1e-4f,
              "클램프");
  });

  runCase("rate 명령도 하드 실링으로 클램프된다", [] {
    YawOuter s = updateYawOuter(500.0f, 0.0f, 0.0f, 0.0f, false, kDead, kSettle);
    CHECK(!s.hold);
    checkNear(yawTargetRateDps(s, 500.0f, 0.0f, 3.0f, 180.0f), 180.0f, 1e-4f,
              "양의 클램프");
    checkNear(yawTargetRateDps(s, -500.0f, 0.0f, 3.0f, 180.0f), -180.0f, 1e-4f,
              "음의 클램프");
  });

  runCase("±180 경계에서 짧은 쪽으로 복원한다", [] {
    YawOuter s = updateYawOuter(0.0f, 1.0f, 179.0f, -179.0f, false, kDead, kSettle);
    CHECK(s.hold);
    // wrapDeg(-179 - 179) = +2 → +6
    checkNear(yawTargetRateDps(s, 0.0f, 179.0f, 3.0f, 180.0f), 6.0f, 1e-3f,
              "짧은 경로");
  });

  if (g_failures != 0) {
    std::cerr << g_failures << " yaw-command case(s) failed\n";
    return 1;
  }
  std::cout << "8/8 yaw-command cases passed\n";
  return 0;
}
