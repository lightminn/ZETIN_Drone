#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "dual_imu_cascade_pwm.ino"

namespace {

int test_count = 0;
int failure_count = 0;

[[noreturn]] void fail(const char *expression, int line, const std::string &detail = {}) {
  std::ostringstream message;
  message << "line " << line << ": " << expression;
  if (!detail.empty()) message << " (" << detail << ")";
  throw std::runtime_error(message.str());
}

#define CHECK(expression) \
  do { \
    if (!(expression)) fail(#expression, __LINE__); \
  } while (false)

template <typename Actual, typename Expected>
void checkEqual(const Actual &actual, const Expected &expected,
                const char *expression, int line) {
  if (!(actual == expected)) {
    std::ostringstream detail;
    detail << "actual=" << actual << ", expected=" << expected;
    fail(expression, line, detail.str());
  }
}

#define CHECK_EQ(actual, expected) \
  checkEqual((actual), (expected), #actual " == " #expected, __LINE__)

void checkNear(double actual, double expected, double tolerance,
               const char *expression, int line) {
  if (!(std::fabs(actual - expected) <= tolerance)) {
    std::ostringstream detail;
    detail << "actual=" << actual << ", expected=" << expected
           << ", tolerance=" << tolerance;
    fail(expression, line, detail.str());
  }
}

#define CHECK_NEAR(actual, expected, tolerance) \
  checkNear((actual), (expected), (tolerance), \
            #actual " ~= " #expected, __LINE__)

void runCase(const std::string &name, const std::function<void()> &body) {
  test_count++;
  try {
    body();
    std::cout << "[PASS] " << name << '\n';
  } catch (const std::exception &error) {
    failure_count++;
    std::cerr << "[FAIL] " << name << ": " << error.what() << '\n';
  }
}

void checkMotors(const MotorMix &mix, int m1, int m2, int m3, int m4) {
  CHECK_EQ(mix.motor[0], m1);
  CHECK_EQ(mix.motor[1], m2);
  CHECK_EQ(mix.motor[2], m3);
  CHECK_EQ(mix.motor[3], m4);
}

void checkAllMotorsInRange(const MotorMix &mix, int low, int high) {
  for (int motor : mix.motor) {
    CHECK(motor >= low);
    CHECK(motor <= high);
  }
}

void sendRc(const std::string &command) {
  std::vector<char> buffer(command.begin(), command.end());
  buffer.push_back('\0');
  handleRcCommand(buffer.data());
}

void sendRcr(const std::string &command) {
  std::vector<char> buffer(command.begin(), command.end());
  buffer.push_back('\0');
  handleRcrCommand(buffer.data());
}

void sendUdpCommandOnce(const std::string &command) {
  wifi_udp_fake::incoming_packet = command;
  arduino_fake::stop_on_task_delay = true;
  try {
    udp_task(nullptr);
  } catch (const arduino_fake::TaskDelayExit &) {
    arduino_fake::stop_on_task_delay = false;
    return;
  } catch (...) {
    arduino_fake::stop_on_task_delay = false;
    throw;
  }
}

void runPidTicks(uint32_t ticks) {
  arduino_fake::tick_index = 0;
  arduino_fake::tick_limit = ticks;
  try {
    pid_task(nullptr);
  } catch (const arduino_fake::TaskDelayExit &) {
  }
  arduino_fake::tick_limit = 0;
}

void resetRcState() {
  lastRcSeq = 0;
  rcSeqValid = false;
  rcTotalPkts = 0;
  rcDroppedPkts = 0;
  targetAngleX = 9.0f;
  targetAngleY = 8.0f;
  targetAngleZ = 7.0f;
  lastRcMs = 6;
  arduino_fake::millis_value = 100;
  targetYawRate = 0.0f;
}

struct GainSlot {
  const char *name;
  volatile float *value;
};

GainSlot gain_slots[] = {
  {"Kp_Angle_Roll", &Kp_Angle_Roll},
  {"Kp_Angle_Pitch", &Kp_Angle_Pitch},
  {"Kp_Angle_Yaw", &Kp_Angle_Yaw},
  {"Kp_Rate_Roll", &Kp_Rate_Roll},
  {"Ki_Rate_Roll", &Ki_Rate_Roll},
  {"Kd_Rate_Roll", &Kd_Rate_Roll},
  {"Kp_Rate_Pitch", &Kp_Rate_Pitch},
  {"Ki_Rate_Pitch", &Ki_Rate_Pitch},
  {"Kd_Rate_Pitch", &Kd_Rate_Pitch},
  {"Kp_Rate_Yaw", &Kp_Rate_Yaw},
  {"Ki_Rate_Yaw", &Ki_Rate_Yaw},
  {"Kd_Rate_Yaw", &Kd_Rate_Yaw},
};

void seedGains() {
  for (std::size_t index = 0; index < std::size(gain_slots); index++) {
    *gain_slots[index].value = 10.0f + static_cast<float>(index);
  }
}

void checkGainCommand(const char *prefix, const std::vector<std::size_t> &changed) {
  seedGains();
  std::vector<float> before;
  for (const auto &slot : gain_slots) {
    before.push_back(static_cast<float>(*slot.value));
  }

  constexpr float commanded = 77.25f;
  const std::string command = std::string(prefix) + " 77.25";
  handleGainCommand(command.c_str());

  for (std::size_t index = 0; index < std::size(gain_slots); index++) {
    bool should_change = false;
    for (std::size_t changed_index : changed) {
      if (changed_index == index) should_change = true;
    }
    const float expected = should_change ? commanded : before[index];
    CHECK_NEAR(*gain_slots[index].value, expected, 1e-6f);
  }
}

inv_imu_sensor_event_t eventWith(
    int16_t gx, int16_t gy, int16_t gz,
    int16_t ax = 0, int16_t ay = 0, int16_t az = 0) {
  inv_imu_sensor_event_t event = {};
  event.gyro[0] = gx;
  event.gyro[1] = gy;
  event.gyro[2] = gz;
  event.accel[0] = ax;
  event.accel[1] = ay;
  event.accel[2] = az;
  return event;
}

}  // namespace

int main() {
  runCase("mix: zero command keeps all motors equal", [] {
    MotorMix mix = mixAndDesaturate(0, 0, 0, 1175, 1050, 1300);
    checkMotors(mix, 1175, 1175, 1175, 1175);
    CHECK(!mix.scaled);
  });

  runCase("mix: pure roll follows FL/RR/FR/RL signs", [] {
    MotorMix mix = mixAndDesaturate(10, 0, 0, 1200, 1050, 1300);
    checkMotors(mix, 1210, 1190, 1190, 1210);
  });

  runCase("mix: pure pitch follows FL/RR/FR/RL signs", [] {
    MotorMix mix = mixAndDesaturate(0, 10, 0, 1200, 1050, 1300);
    checkMotors(mix, 1190, 1210, 1190, 1210);
  });

  runCase("mix: pure yaw follows FL/RR/FR/RL signs", [] {
    MotorMix mix = mixAndDesaturate(0, 0, 10, 1200, 1050, 1300);
    checkMotors(mix, 1190, 1190, 1210, 1210);
  });

  runCase("mix: extreme commands remain inside motor limits", [] {
    MotorMix mix = mixAndDesaturate(100000, -200000, 300000,
                                     5000, 1050, 1300);
    checkAllMotorsInRange(mix, 1050, 1300);
  });

  runCase("mix: fitting commands preserve exact torque differences", [] {
    MotorMix mix = mixAndDesaturate(10, 20, 5, 1175, 1050, 1300);
    checkMotors(mix, 1160, 1180, 1150, 1210);
    CHECK(!mix.scaled);
    CHECK_EQ(mix.motor[0] - mix.motor[1], -20);
    CHECK_EQ(mix.motor[2] - mix.motor[3], -60);
  });

  runCase("mix: saturation applies one common attitude scale", [] {
    MotorMix mix = mixAndDesaturate(120, 60, 30, 1100, 1000, 1240);
    checkMotors(mix, 1120, 1040, 1000, 1240);
    CHECK(mix.scaled);
    CHECK_NEAR(
        static_cast<float>(mix.motor[0] - mix.motor[1]) / (30.0f - -90.0f),
        2.0f / 3.0f, 1e-6f);
    CHECK_NEAR(
        static_cast<float>(mix.motor[1] - mix.motor[2]) / (-90.0f - -150.0f),
        2.0f / 3.0f, 1e-6f);
    CHECK_NEAR(
        static_cast<float>(mix.motor[3] - mix.motor[2]) / (210.0f - -150.0f),
        2.0f / 3.0f, 1e-6f);
  });

  runCase("mix: collective shifts before attitude scaling", [] {
    MotorMix mix = mixAndDesaturate(30, 0, 0, 1040, 1050, 1250);
    checkMotors(mix, 1110, 1050, 1050, 1110);
    CHECK(!mix.scaled);
  });

  runCase("mix: minMotor above maxMotor collapses to a safe bound", [] {
    MotorMix mix = mixAndDesaturate(30, 0, 0, 1500, 1800, 1200);
    checkMotors(mix, 1800, 1800, 1800, 1800);
    CHECK(mix.scaled);
  });

  runCase("mix: out-of-range limits and throttle are constrained", [] {
    MotorMix low = mixAndDesaturate(0, 0, 0, -100000, 500, 2500);
    MotorMix high = mixAndDesaturate(0, 0, 0, 100000, 500, 2500);
    checkMotors(low, 1000, 1000, 1000, 1000);
    checkMotors(high, 2000, 2000, 2000, 2000);
  });

  for (int us : {1000, 1500, 2000}) {
    runCase("writeMotor: " + std::to_string(us) + " us maps to 14-bit duty", [us] {
      arduino_fake::ledc_duty_by_pin.clear();
      writeMotor(pinM1, us);
      const uint32_t expected = (static_cast<uint32_t>(us) * 16383U) / 2500U;
      CHECK_EQ(arduino_fake::ledc_duty_by_pin.at(pinM1), expected);
    });
  }

  runCase("writeMotor: values below 1000 us are constrained first", [] {
    arduino_fake::ledc_duty_by_pin.clear();
    writeMotor(pinM2, -500);
    CHECK_EQ(arduino_fake::ledc_duty_by_pin.at(pinM2), 1000U * 16383U / 2500U);
  });

  runCase("writeMotor: values above 2000 us are constrained first", [] {
    arduino_fake::ledc_duty_by_pin.clear();
    writeMotor(pinM3, 9000);
    CHECK_EQ(arduino_fake::ledc_duty_by_pin.at(pinM3), 2000U * 16383U / 2500U);
  });

  runCase("RC watchdog: timestamp slightly in the future is not timed out", [] {
    CHECK(!rcTimedOut(1000U, 1001U));
  });

  runCase("RC watchdog: timestamp far in the future is not timed out", [] {
    CHECK(!rcTimedOut(1000U, 5000U));
  });

  runCase("RC watchdog: exact timeout boundary is not timed out", [] {
    CHECK(!rcTimedOut(1000U + RC_TIMEOUT_MS, 1000U));
  });

  runCase("RC watchdog: one millisecond past timeout is timed out", [] {
    CHECK(rcTimedOut(1000U + RC_TIMEOUT_MS + 1U, 1000U));
  });

  runCase("RC watchdog: zero age is not timed out", [] {
    CHECK(!rcTimedOut(1000U, 1000U));
  });

  runCase("RC watchdog: short age across millis wrap is not timed out", [] {
    CHECK(!rcTimedOut(10U, 0xFFFFFFF0U));
  });

  runCase("RC watchdog: expired age across millis wrap is timed out", [] {
    CHECK(rcTimedOut(1000U, 0xFFFFFF00U));
  });

  runCase("RC: valid sequenced packet updates counters and targets", [] {
    resetRcState();
    sendRc("rc 10 1.5 -2.5 3.5");
    CHECK_EQ(lastRcSeq, 10U);
    CHECK_EQ(rcTotalPkts, 1U);
    CHECK_EQ(rcDroppedPkts, 0U);
    CHECK_NEAR(targetAngleX, 1.5f, 1e-6f);
    CHECK_NEAR(targetAngleY, -2.5f, 1e-6f);
    CHECK_NEAR(targetAngleZ, 3.5f, 1e-6f);
    CHECK_EQ(lastRcMs, 100U);
  });

  runCase("RC: duplicate sequence is counted and discarded", [] {
    resetRcState();
    sendRc("rc 10 1 2 3");
    arduino_fake::millis_value = 200;
    sendRc("rc 10 9 9 9");
    CHECK_EQ(lastRcSeq, 10U);
    CHECK_EQ(rcTotalPkts, 2U);
    CHECK_EQ(rcDroppedPkts, 1U);
    CHECK_NEAR(targetAngleX, 1.0f, 1e-6f);
    CHECK_EQ(lastRcMs, 100U);
  });

  runCase("RC: stale sequence is counted and discarded", [] {
    resetRcState();
    sendRc("rc 10 1 2 3");
    arduino_fake::millis_value = 200;
    sendRc("rc 9 9 9 9");
    CHECK_EQ(lastRcSeq, 10U);
    CHECK_EQ(rcTotalPkts, 2U);
    CHECK_EQ(rcDroppedPkts, 1U);
    CHECK_NEAR(targetAngleX, 1.0f, 1e-6f);
    CHECK_EQ(lastRcMs, 100U);
  });

  runCase("RC: sequence gap adds N minus one dropped packets", [] {
    resetRcState();
    sendRc("rc 10 1 2 3");
    sendRc("rc 14 4 5 6");
    CHECK_EQ(lastRcSeq, 14U);
    CHECK_EQ(rcTotalPkts, 2U);
    CHECK_EQ(rcDroppedPkts, 3U);
    CHECK_NEAR(targetAngleX, 4.0f, 1e-6f);
  });

  runCase("RC: uint32 sequence wrap is forward progress", [] {
    resetRcState();
    lastRcSeq = std::numeric_limits<uint32_t>::max() - 1U;
    rcSeqValid = true;
    sendRc("rc 4294967295 1 2 3");
    sendRc("rc 0 4 5 6");
    CHECK_EQ(lastRcSeq, 0U);
    CHECK_EQ(rcTotalPkts, 2U);
    CHECK_EQ(rcDroppedPkts, 0U);
    CHECK_NEAR(targetAngleX, 4.0f, 1e-6f);
  });

  runCase("RC: duplicate zero after wrap is counted and discarded", [] {
    resetRcState();
    lastRcSeq = std::numeric_limits<uint32_t>::max();
    rcSeqValid = true;
    sendRc("rc 0 1 2 3");
    arduino_fake::millis_value = 200;
    sendRc("rc 0 9 9 9");
    CHECK_EQ(lastRcSeq, 0U);
    CHECK_EQ(rcTotalPkts, 2U);
    CHECK_EQ(rcDroppedPkts, 1U);
    CHECK_NEAR(targetAngleX, 1.0f, 1e-6f);
    CHECK_EQ(lastRcMs, 100U);
  });

  runCase("RC: gap immediately after wrapped zero is accounted", [] {
    resetRcState();
    lastRcSeq = std::numeric_limits<uint32_t>::max();
    rcSeqValid = true;
    sendRc("rc 0 1 2 3");
    sendRc("rc 2 4 5 6");
    CHECK_EQ(lastRcSeq, 2U);
    CHECK_EQ(rcTotalPkts, 2U);
    CHECK_EQ(rcDroppedPkts, 1U);
    CHECK_NEAR(targetAngleX, 4.0f, 1e-6f);
  });

  runCase("RC: two-field bench form is accepted", [] {
    resetRcState();
    sendRc("rc -4 5");
    CHECK_NEAR(targetAngleX, -4.0f, 1e-6f);
    CHECK_NEAR(targetAngleY, 5.0f, 1e-6f);
    CHECK_NEAR(targetAngleZ, 7.0f, 1e-6f);
    CHECK_EQ(rcTotalPkts, 0U);
    CHECK_EQ(lastRcMs, 100U);
  });

  runCase("RC: three-field bench form is accepted", [] {
    resetRcState();
    sendRc("rc -4 5 6");
    CHECK_NEAR(targetAngleX, -4.0f, 1e-6f);
    CHECK_NEAR(targetAngleY, 5.0f, 1e-6f);
    CHECK_NEAR(targetAngleZ, 6.0f, 1e-6f);
    CHECK_EQ(rcTotalPkts, 0U);
  });

  runCase("RC: fifth argument is rejected", [] {
    resetRcState();
    sendRc("rc 10 1 2 3 extra");
    CHECK_EQ(lastRcSeq, 0U);
    CHECK_EQ(rcTotalPkts, 0U);
    CHECK_NEAR(targetAngleX, 9.0f, 1e-6f);
    CHECK_EQ(lastRcMs, 6U);
  });

  runCase("RC: malformed numeric fields are rejected", [] {
    resetRcState();
    sendRc("rc nope 1 2 3");
    sendRc("rc 10 1oops 2 3");
    sendRc("rc x 2");
    CHECK_EQ(lastRcSeq, 0U);
    CHECK_EQ(rcTotalPkts, 0U);
    CHECK_NEAR(targetAngleX, 9.0f, 1e-6f);
    CHECK_EQ(lastRcMs, 6U);
  });

  runCase("RC: sequences with a leading sign are rejected", [] {
    resetRcState();
    sendRc("rc -1 1 2 3");
    sendRc("rc +1 1 2 3");
    CHECK_EQ(lastRcSeq, 0U);
    CHECK_EQ(rcTotalPkts, 0U);
    CHECK_NEAR(targetAngleX, 9.0f, 1e-6f);
    CHECK_EQ(lastRcMs, 6U);
  });

  runCase("RC: sequence above uint32 range is rejected", [] {
    resetRcState();
    sendRc("rc 4294967296 1 2 3");
    CHECK_EQ(lastRcSeq, 0U);
    CHECK_EQ(rcTotalPkts, 0U);
    CHECK_NEAR(targetAngleX, 9.0f, 1e-6f);
    CHECK_EQ(lastRcMs, 6U);
  });

  runCase("RC: start resets sequence validity before the next baseline", [] {
    resetRcState();
    lastRcSeq = 77U;
    rcSeqValid = true;
    calibration_ok = true;
    angleX = angleY = 0.0f;
    imu1_frozen_now = imu2_frozen_now = false;
    imu_disagree_now = false;
    sendUdpCommandOnce("start");
    CHECK_EQ(lastRcSeq, 0U);
    CHECK(!rcSeqValid);

    sendRc("rc 100 1 2 3");
    CHECK_EQ(rcTotalPkts, 1U);
    CHECK_EQ(rcDroppedPkts, 0U);
    CHECK_EQ(lastRcSeq, 100U);
    CHECK(rcSeqValid);
  });

  runCase("trim: 절대값이 저장된다", [] {
    trim_roll = 0.0f; trim_pitch = 0.0f;
    sendUdpCommandOnce("trim 1.5 -2.0");
    CHECK_NEAR(trim_roll, 1.5f, 1e-4f);
    CHECK_NEAR(trim_pitch, -2.0f, 1e-4f);
  });

  runCase("trim: +-TRIM_MAX_DEG로 클램프된다", [] {
    trim_roll = 0.0f; trim_pitch = 0.0f;
    sendUdpCommandOnce("trim 50 -50");
    CHECK_NEAR(trim_roll, TRIM_MAX_DEG, 1e-4f);
    CHECK_NEAR(trim_pitch, -TRIM_MAX_DEG, 1e-4f);
  });

  runCase("trim: 잘못된 입력은 무시된다", [] {
    trim_roll = 1.0f; trim_pitch = 2.0f;
    sendUdpCommandOnce("trim abc 1");
    sendUdpCommandOnce("trim 1");
    sendUdpCommandOnce("trim 1 2 3");
    CHECK_NEAR(trim_roll, 1.0f, 1e-4f);
    CHECK_NEAR(trim_pitch, 2.0f, 1e-4f);
  });

  runCase("trim: 무장 중에도 수락된다", [] {
    trim_roll = 0.0f;
    safety_lock = false;
    sendUdpCommandOnce("trim 3 0");
    CHECK_NEAR(trim_roll, 3.0f, 1e-4f);
    safety_lock = true;
  });

  runCase("목표각은 트림을 더한 뒤 +-30도로 클램프된다", [] {
    resetRcState();
    trim_roll = 10.0f; trim_pitch = -10.0f;
    sendRcr("rcr 1 25 -25 0");
    // 25 + 10 = 35 -> 30 으로 클램프 (35가 아니다)
    CHECK_NEAR(targetAngleX, 30.0f, 1e-4f);
    CHECK_NEAR(targetAngleY, -30.0f, 1e-4f);
    trim_roll = 0.0f; trim_pitch = 0.0f;
  });

  runCase("트림이 목표각에 실제로 더해진다", [] {
    resetRcState();
    trim_roll = 2.0f; trim_pitch = -1.0f;
    sendRcr("rcr 1 0 0 0");
    CHECK_NEAR(targetAngleX, 2.0f, 1e-4f);
    CHECK_NEAR(targetAngleY, -1.0f, 1e-4f);
    trim_roll = 0.0f; trim_pitch = 0.0f;
  });

  runCase("start와 stop은 트림을 지우지 않는다", [] {
    trim_roll = 4.0f; trim_pitch = -3.0f;
    calibration_ok = true;
    safety_lock = true;
    sendUdpCommandOnce("start");
    CHECK_NEAR(trim_roll, 4.0f, 1e-4f);
    sendUdpCommandOnce("stop");
    CHECK_NEAR(trim_roll, 4.0f, 1e-4f);
    CHECK_NEAR(trim_pitch, -3.0f, 1e-4f);
    trim_roll = 0.0f; trim_pitch = 0.0f;
  });

  runCase("RC 타임아웃이 즉시 컷이 아니라 자동착륙으로 간다", [] {
    calibration_ok = true;
    safety_lock = true;
    sendUdpCommandOnce("start");
    CHECK(!safety_lock);
    fs_phase = FS_NONE;
    base_throttle = 1360;

    // rc가 끊긴 지 오래된 상태를 만든다
    lastRcMs = 0;
    arduino_fake::millis_value = RC_TIMEOUT_MS + 100;
    runPidTicks(1);

    CHECK_EQ((int)fs_phase, (int)FS_DESCENDING);
    CHECK(!safety_lock);                       // 아직 컷이 아니다
    CHECK_EQ(base_throttle, 1360 - FS_DESCENT_DELTA_US);
  });

  runCase("stop은 자동착륙 중에도 즉시 컷이다", [] {
    fs_phase = FS_DESCENDING;
    safety_lock = false;
    sendUdpCommandOnce("stop");
    CHECK(safety_lock);
    CHECK_EQ(base_throttle, 1000);
  });

  runCase("start가 fs_phase를 FS_NONE으로 되돌린다", [] {
    fs_phase = FS_CUT_TIMEOUT;
    calibration_ok = true;
    safety_lock = true;
    sendUdpCommandOnce("start");
    CHECK_EQ((int)fs_phase, (int)FS_NONE);
  });

  runCase("회귀: 하강 목표는 0이 아니라 트림값이다", [] {
    // 사용자가 잡은 결함. 0을 명령하면 트림이 보정하던 흐름이 그대로 재현돼
    // 5초에 수 미터를 흘러간다(설계 문서 §3).
    trim_roll = 2.0f; trim_pitch = -1.5f;
    calibration_ok = true;
    safety_lock = true;
    sendUdpCommandOnce("start");
    fs_phase = FS_NONE;
    base_throttle = 1360;
    lastRcMs = 0;
    arduino_fake::millis_value = RC_TIMEOUT_MS + 100;
    runPidTicks(1);

    CHECK_EQ((int)fs_phase, (int)FS_DESCENDING);
    CHECK_NEAR(targetAngleX, 2.0f, 1e-4f);
    CHECK_NEAR(targetAngleY, -1.5f, 1e-4f);
    trim_roll = 0.0f; trim_pitch = 0.0f;
  });

  runCase("회귀: 하강 중 rc가 돌아와도 제어권이 복귀하지 않는다", [] {
    // 매 tick 덮어쓰기가 없으면 rc 파서가 targetAngle을 다시 써서
    // '자동 복귀 없음' 결정과 어긋난다(설계 문서 §1).
    trim_roll = 0.0f; trim_pitch = 0.0f;
    calibration_ok = true;
    safety_lock = true;
    sendUdpCommandOnce("start");
    fs_phase = FS_DESCENDING;
    base_throttle = 1360;
    lastRcMs = 0;
    arduino_fake::millis_value = RC_TIMEOUT_MS + 100;
    runPidTicks(1);

    resetRcState();
    sendRcr("rcr 1 25 -25 80");        // 링크 복귀: 큰 조종 입력
    runPidTicks(1);

    CHECK_NEAR(targetAngleX, 0.0f, 1e-4f);   // 스틱 입력이 무시된다
    CHECK_NEAR(targetAngleY, 0.0f, 1e-4f);
    CHECK_NEAR(targetYawRate, 0.0f, 1e-4f);
    CHECK_EQ((int)fs_phase, (int)FS_DESCENDING);   // 상태도 그대로
  });

  runCase("회귀: 진입 로그는 하강 내내 딱 한 번만 나간다", [] {
    // 가드가 없으면 rcTimedOut이 참인 동안 1kHz로 재실행돼 Serial.println이
    // 115200 baud TX 버퍼를 포화시키고, pid_task가 블로킹되면 500ms 태스크
    // 워치독이 비행 중 재부팅을 일으킨다. 이 태스크에서 가장 위험한 실패 모드다.
    calibration_ok = true;
    safety_lock = true;
    sendUdpCommandOnce("start");
    fs_phase = FS_NONE;
    base_throttle = 1360;
    lastRcMs = 0;
    arduino_fake::millis_value = RC_TIMEOUT_MS + 100;

    arduino_fake::serial_output.clear();
    runPidTicks(50);

    const std::string &log = arduino_fake::serial_output;
    std::size_t hits = 0;
    for (std::size_t at = log.find("AUTO-LAND");
         at != std::string::npos;
         at = log.find("AUTO-LAND", at + 1)) {
      hits++;
    }
    CHECK_EQ(hits, static_cast<std::size_t>(1));
  });

  runCase("자동착륙 종료 후에는 재시동 없이 날 수 없다", [] {
    fs_phase = FS_DESCENDING;
    safety_lock = false;
    base_throttle = 1360;
    lastRcMs = 0;
    arduino_fake::millis_value = RC_TIMEOUT_MS + 100 + FS_MAX_MS;
    runPidTicks(2);
    CHECK(safety_lock);                        // 백스톱으로 잠겼다
    CHECK_EQ(motorOut[0], 1000);
  });

  runCase("rcr: 정상 패킷이 yaw 각속도와 roll/pitch를 설정한다", [] {
    resetRcState();
    sendRcr("rcr 1 5.5 -6.5 45.0");
    CHECK_NEAR(targetAngleX, 5.5f, 1e-4f);
    CHECK_NEAR(targetAngleY, -6.5f, 1e-4f);
    CHECK_NEAR(targetYawRate, 45.0f, 1e-4f);
  });

  runCase("rcr: roll/pitch는 +-30도로 클램프된다", [] {
    resetRcState();
    sendRcr("rcr 1 90 -90 0");
    CHECK_NEAR(targetAngleX, 30.0f, 1e-4f);
    CHECK_NEAR(targetAngleY, -30.0f, 1e-4f);
  });

  runCase("rcr: yaw 각속도는 +-180dps로 클램프된다", [] {
    resetRcState();
    sendRcr("rcr 1 0 0 500");
    CHECK_NEAR(targetYawRate, 180.0f, 1e-4f);
    sendRcr("rcr 2 0 0 -500");
    CHECK_NEAR(targetYawRate, -180.0f, 1e-4f);
  });

  runCase("rcr: 역순/중복 seq는 폐기되고 드롭으로 계수된다", [] {
    resetRcState();
    sendRcr("rcr 10 0 0 20");
    const uint32_t dropped_before = rcDroppedPkts;
    sendRcr("rcr 9 0 0 99");
    CHECK_NEAR(targetYawRate, 20.0f, 1e-4f);
    CHECK_EQ(rcDroppedPkts, dropped_before + 1U);
  });

  runCase("rcr: seq 건너뜀이 드롭 수에 반영된다", [] {
    resetRcState();
    sendRcr("rcr 10 0 0 0");
    sendRcr("rcr 14 0 0 0");
    CHECK_EQ(rcDroppedPkts, 3U);
  });

  runCase("rcr: 인자 수 불일치와 비수치는 거부된다", [] {
    resetRcState();
    sendRcr("rcr 1 0 0 30");
    CHECK_NEAR(targetYawRate, 30.0f, 1e-4f);
    sendRcr("rcr 2 0 0");          // 인자 부족
    CHECK_NEAR(targetYawRate, 30.0f, 1e-4f);
    sendRcr("rcr 3 0 0 abc");      // 비수치
    CHECK_NEAR(targetYawRate, 30.0f, 1e-4f);
    sendRcr("rcr 4 0 0 10 99");    // 여분 필드
    CHECK_NEAR(targetYawRate, 30.0f, 1e-4f);
  });

  runCase("rcr: udp_task 디스패치가 rc와 구분해서 처리한다", [] {
    resetRcState();
    sendUdpCommandOnce("rcr 1 0 0 60");
    CHECK_NEAR(targetYawRate, 60.0f, 1e-4f);
  });

  struct GainCommandCase {
    const char *prefix;
    std::vector<std::size_t> changed;
  };
  const std::vector<GainCommandCase> gain_cases = {
    {"rp", {3, 6}}, {"ri", {4, 7}}, {"rd", {5, 8}},
    {"ap", {0, 1}}, {"ar", {0}}, {"at", {1}}, {"ay", {2}},
    {"yp", {9}}, {"yi", {10}}, {"yd", {11}},
    {"pa", {3, 6}}, {"ia", {4, 7}}, {"da", {5, 8}},
    {"pr", {3}}, {"ir", {4}}, {"dr", {5}},
    {"pp", {6}}, {"ip", {7}}, {"dp", {8}},
    {"py", {9}}, {"iy", {10}}, {"dy", {11}},
  };
  for (const auto &gain_case : gain_cases) {
    runCase(std::string("gain: ") + gain_case.prefix + " writes only its target",
            [gain_case] { checkGainCommand(gain_case.prefix, gain_case.changed); });
  }

  runCase("gain: negative values are rejected", [] {
    seedGains();
    const float before = Kp_Rate_Pitch;
    handleGainCommand("pp -0.01");
    CHECK_NEAR(Kp_Rate_Pitch, before, 1e-6f);
  });

  runCase("gain: values above 100 are rejected", [] {
    seedGains();
    const float before = Kp_Rate_Pitch;
    handleGainCommand("pp 100.01");
    CHECK_NEAR(Kp_Rate_Pitch, before, 1e-6f);
  });

  runCase("gain: lower boundary zero is accepted", [] {
    seedGains();
    handleGainCommand("pp 0");
    CHECK_NEAR(Kp_Rate_Pitch, 0.0f, 1e-6f);
  });

  runCase("gain: upper boundary 100 is accepted", [] {
    seedGains();
    handleGainCommand("pp 100");
    CHECK_NEAR(Kp_Rate_Pitch, 100.0f, 1e-6f);
  });

  runCase("gain: malformed values leave gains unchanged", [] {
    seedGains();
    const float before = Kp_Rate_Pitch;
    handleGainCommand("pp 2.5oops");
    CHECK_NEAR(Kp_Rate_Pitch, before, 1e-6f);
  });

  runCase("parseFloatStrict: trailing garbage is rejected", [] {
    float value = 9.0f;
    CHECK(!parseFloatStrict("2.5oops", value));
    CHECK_NEAR(value, 9.0f, 1e-6f);
  });

  runCase("parseFloatStrict: trailing spaces and tabs are accepted", [] {
    float value = 0.0f;
    CHECK(parseFloatStrict("2.5 \t", value));
    CHECK_NEAR(value, 2.5f, 1e-6f);
  });

  runCase("parseFloatStrict: nan and infinity are rejected", [] {
    float value = 9.0f;
    CHECK(!parseFloatStrict("nan", value));
    CHECK(!parseFloatStrict("inf", value));
    CHECK(!parseFloatStrict("-inf", value));
    CHECK_NEAR(value, 9.0f, 1e-6f);
  });

  runCase("parseFloatStrict: empty input is rejected", [] {
    float value = 9.0f;
    CHECK(!parseFloatStrict("", value));
    CHECK_NEAR(value, 9.0f, 1e-6f);
  });

  runCase("parseIntStrict: trailing garbage is rejected", [] {
    long value = 9;
    CHECK(!parseIntStrict("12oops", value));
    CHECK_EQ(value, 9L);
  });

  runCase("parseIntStrict: trailing spaces and tabs are accepted", [] {
    long value = 0;
    CHECK(parseIntStrict("12 \t", value));
    CHECK_EQ(value, 12L);
  });

  runCase("parseIntStrict: empty input is rejected", [] {
    long value = 9;
    CHECK(!parseIntStrict("", value));
    CHECK_EQ(value, 9L);
  });

  runCase("parseIntStrict: ESP32 long maximum is accepted", [] {
    long value = 0;
    CHECK(parseIntStrict("2147483647", value));
    CHECK_EQ(value, 2147483647L);
  });

  runCase("parseIntStrict: above ESP32 long maximum saturates at LONG_MAX", [] {
    long value = 9;
    CHECK(parseIntStrict("2147483648", value));
    CHECK_EQ(value, std::numeric_limits<long>::max());
  });

  runCase("parseIntStrict: below ESP32 long minimum saturates at LONG_MIN", [] {
    long value = 9;
    CHECK(parseIntStrict("-2147483649", value));
    CHECK_EQ(value, std::numeric_limits<long>::min());
  });

  runCase("parseIntStrict: strtol overflow saturates at LONG_MAX", [] {
    long value = 9;
    CHECK(parseIntStrict("999999999999999999999999999999", value));
    CHECK_EQ(value, std::numeric_limits<long>::max());
  });

  runCase("parseIntStrict: saturation is safe at th and yaw call sites", [] {
    base_throttle = 1100;
    min_throttle = 1050;
    max_throttle = 1250;
    yaw_hold_override = false;
    angleZ = 42.0f;
    targetAngleZ = -7.0f;

    sendUdpCommandOnce("th 999999999999999999999999999999");
    CHECK_EQ(base_throttle, 1900);
    CHECK_EQ(min_throttle, 1750);
    CHECK_EQ(max_throttle, 1900);

    sendUdpCommandOnce("yaw 999999999999999999999999999999");
    CHECK(!yaw_hold_override);
    CHECK_NEAR(targetAngleZ, -7.0f, 1e-6f);
  });

  runCase("yaw 1은 무장 중 거부되고 yaw 0은 언제나 수락된다", [] {
    yaw_hold_override = false;
    safety_lock = false;                       // 무장 상태
    sendUdpCommandOnce("yaw 1");
    CHECK(!yaw_hold_override);                 // 거부

    safety_lock = true;                        // 시동 해제 상태
    sendUdpCommandOnce("yaw 1");
    CHECK(yaw_hold_override);                  // 수락

    safety_lock = false;
    sendUdpCommandOnce("yaw 0");
    CHECK(!yaw_hold_override);                 // 끄기는 무장 중에도 허용
  });

  runCase("잠금 진입 헬퍼는 오버라이드를 엣지에서 1회만 해제한다", [] {
    yaw_hold_override = true;
    bool wasLocked = false;
    enterLockedState(wasLocked);
    CHECK(!yaw_hold_override);      // 무장 해제 엣지에서 해제
    CHECK(wasLocked);

    // 잠긴 상태가 유지되는 동안에는 다시 켤 수 있어야 한다
    // (매 tick 지우면 시동 해제 상태에서 yaw 1을 켤 수 없다)
    yaw_hold_override = true;
    enterLockedState(wasLocked);
    CHECK(yaw_hold_override);
  });

  runCase("mag command enables lazily and disables without changing yaw", [] {
    safety_lock = true;  // 최초 lazy init(블로킹 I2C)은 disarmed에서만 허용된다.
    mag_enabled = false;
    mag_ready = false;
    angleZ = 42.0f;
    bmm.begin_result = 0;

    sendUdpCommandOnce("mag 1");
    CHECK(mag_enabled);
    CHECK(mag_ready);
    CHECK_NEAR(angleZ, 42.0f, 1e-6f);

    sendUdpCommandOnce("mag 0");
    CHECK(!mag_enabled);
    CHECK_NEAR(angleZ, 42.0f, 1e-6f);
  });

  runCase("mag enable refused while armed before init", [] {
    safety_lock = false;  // armed
    mag_enabled = false;
    mag_ready = false;
    bmm.begin_result = 0;
    arduino_fake::serial_output.clear();

    sendUdpCommandOnce("mag 1");

    CHECK(!mag_enabled);
    CHECK(!mag_ready);  // 블로킹 init이 실행되지 않아야 한다.
    CHECK(arduino_fake::serial_output.find(
              "Mag refused (armed, not initialized)") != std::string::npos);
    safety_lock = true;
  });

  runCase("mag command rejects values other than zero and one", [] {
    mag_enabled = false;
    sendUdpCommandOnce("mag 2");
    CHECK(!mag_enabled);
    sendUdpCommandOnce("mag -1");
    CHECK(!mag_enabled);
  });

  runCase("magcal collects extrema and prints hard-iron offsets", [] {
    safety_lock = true;
    mag_enabled = false;
    mag_ready = false;
    mag_calibrating = false;
    bmm.begin_result = 0;
    arduino_fake::serial_output.clear();

    sendUdpCommandOnce("magcal 1");
    CHECK(mag_calibrating);
    CHECK(!mag_enabled);

    bmm.next_data.float_x = 10.0f;
    bmm.next_data.float_y = -4.0f;
    bmm.next_data.float_z = 8.0f;
    sampleMagnetometer(20U);
    bmm.next_data.float_x = -2.0f;
    bmm.next_data.float_y = 8.0f;
    bmm.next_data.float_z = -6.0f;
    sampleMagnetometer(40U);

    arduino_fake::serial_output.clear();
    sendUdpCommandOnce("magcal 0");
    CHECK(!mag_calibrating);
    CHECK(arduino_fake::serial_output.find(
              "MAG_HARD_IRON_OFFSET_X = 4.000000f") != std::string::npos);
    CHECK(arduino_fake::serial_output.find(
              "MAG_HARD_IRON_OFFSET_Y = 2.000000f") != std::string::npos);
    CHECK(arduino_fake::serial_output.find(
              "MAG_HARD_IRON_OFFSET_Z = 1.000000f") != std::string::npos);
  });

  runCase("magcal is refused while armed", [] {
    safety_lock = false;
    mag_calibrating = false;
    mag_ready = false;
    bmm.begin_result = 0;
    arduino_fake::serial_output.clear();

    sendUdpCommandOnce("magcal 1");

    CHECK(!mag_calibrating);
    CHECK(!mag_ready);
    CHECK(arduino_fake::serial_output.find(
              "Magcal refused (armed)") != std::string::npos);
    safety_lock = true;
  });

  runCase("start is refused while magcal is active", [] {
    safety_lock = true;
    calibration_ok = true;
    angleX = 0.0f;
    angleY = 0.0f;
    imu1_frozen_now = false;
    imu2_frozen_now = false;
    imu_disagree_now = false;
    mag_calibrating = true;

    sendUdpCommandOnce("start");

    CHECK(safety_lock);
    mag_calibrating = false;
  });

  runCase("compute_alpha: below soft threshold is static", [] {
    CHECK_EQ(compute_alpha(1.0f - ACC_DEV_SOFT * 0.5f, 0, 0), ALPHA_STATIC);
  });

  runCase("compute_alpha: soft boundary selects normal", [] {
    CHECK_EQ(compute_alpha(1.0f - ACC_DEV_SOFT, 0, 0), ALPHA_NORMAL);
  });

  runCase("compute_alpha: between thresholds is normal", [] {
    CHECK_EQ(compute_alpha(1.0f - 0.20f, 0, 0), ALPHA_NORMAL);
  });

  runCase("compute_alpha: hard boundary selects dynamic", [] {
    CHECK_EQ(compute_alpha(1.0f - ACC_DEV_HARD, 0, 0), ALPHA_DYN);
  });

  runCase("compute_alpha: above hard threshold is dynamic", [] {
    CHECK_EQ(compute_alpha(1.0f - 0.40f, 0, 0), ALPHA_DYN);
  });

  runCase("checkFreeze: first call initializes without freezing", [] {
    FreezeMon monitor;
    CHECK(!checkFreeze(monitor, eventWith(1, 2, 3, 4, 5, 6), 10));
    CHECK(monitor.init);
    CHECK_EQ(monitor.since, 0U);
  });

  runCase("checkFreeze: repeated sub-threshold deltas freeze after timeout", [] {
    FreezeMon monitor;
    auto event = eventWith(1, 2, 3, 4, 5, 6);
    CHECK(!checkFreeze(monitor, event, 10));
    CHECK(!checkFreeze(monitor, event, 100));
    CHECK(!checkFreeze(monitor, event, 399));
    CHECK(checkFreeze(monitor, event, 400));
  });

  runCase("checkFreeze: real movement resets the timer", [] {
    FreezeMon monitor;
    auto still = eventWith(10, 20, 30);
    auto moved = eventWith(12, 20, 30);
    CHECK(!checkFreeze(monitor, still, 10));
    CHECK(!checkFreeze(monitor, still, 100));
    CHECK(!checkFreeze(monitor, moved, 350));
    CHECK_EQ(monitor.since, 0U);
    CHECK(!checkFreeze(monitor, moved, 400));
    CHECK(!checkFreeze(monitor, moved, 699));
    CHECK(checkFreeze(monitor, moved, 700));
  });

  runCase("checkFreeze: cancelling per-axis magnitudes still count as movement", [] {
    FreezeMon monitor;
    auto first = eventWith(10, 20, 0);
    auto cancelling = eventWith(11, 19, 0);
    CHECK(!checkFreeze(monitor, first, 10));
    CHECK(!checkFreeze(monitor, first, 100));
    CHECK(!checkFreeze(monitor, cancelling, 350));
    CHECK_EQ(monitor.since, 0U);
    CHECK(!checkFreeze(monitor, cancelling, 400));
    CHECK(!checkFreeze(monitor, cancelling, 450));
  });

  runCase("LowPassFilter: alpha follows dt over rc plus dt", [] {
    constexpr float cutoff = 40.0f;
    constexpr float dt = 0.001f;
    LowPassFilter filter(cutoff, dt);
    const float rc = 1.0f / (2.0f * PI * cutoff);
    CHECK_NEAR(filter.alpha, dt / (rc + dt), 1e-7f);
  });

  runCase("LowPassFilter: reset sets state and default reset clears it", [] {
    LowPassFilter filter(40.0f, 0.001f);
    filter.update(10.0f);
    filter.reset(5.0f);
    CHECK_NEAR(filter.last, 5.0f, 1e-6f);
    CHECK_NEAR(filter.update(5.0f), 5.0f, 1e-6f);
    filter.reset();
    CHECK_NEAR(filter.last, 0.0f, 1e-6f);
  });

  std::cout << "\n" << (test_count - failure_count) << "/" << test_count
            << " native control-math cases passed\n";
  return failure_count == 0 ? 0 : 1;
}
