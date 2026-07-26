#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "dual_imu_cascade_pwm.ino"

namespace {

constexpr double kDt = 0.001;
constexpr double kDegToRad = PI / 180.0;
constexpr double kRadToDeg = 180.0 / PI;

#ifdef SIL_INJECT_SIGN_FAULT
constexpr bool kInjectRollSignFault = true;
#else
constexpr bool kInjectRollSignFault = false;
#endif

int test_count = 0;
int failure_count = 0;
int report_count = 0;
int report_failure_count = 0;

[[noreturn]] void fail(const char *expression, int line,
                       const std::string &detail = {}) {
  std::ostringstream message;
  message << "line " << line << ": " << expression;
  if (!detail.empty()) message << " (" << detail << ")";
  throw std::runtime_error(message.str());
}

#define CHECK(expression) \
  do { \
    if (!(expression)) fail(#expression, __LINE__); \
  } while (false)

#define CHECK_MSG(expression, detail) \
  do { \
    if (!(expression)) fail(#expression, __LINE__, (detail)); \
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

void checkAtMost(double actual, double limit,
                 const char *expression, int line) {
  if (!(actual <= limit)) {
    std::ostringstream detail;
    detail << "actual=" << actual << ", limit=" << limit;
    fail(expression, line, detail.str());
  }
}

#define CHECK_LE(actual, limit) \
  checkAtMost((actual), (limit), #actual " <= " #limit, __LINE__)

void checkAtLeast(double actual, double limit,
                  const char *expression, int line) {
  if (!(actual >= limit)) {
    std::ostringstream detail;
    detail << "actual=" << actual << ", limit=" << limit;
    fail(expression, line, detail.str());
  }
}

#define CHECK_GE(actual, limit) \
  checkAtLeast((actual), (limit), #actual " >= " #limit, __LINE__)

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

void runReport(const std::string &name, const std::function<void()> &body) {
  report_count++;
  try {
    body();
    std::cout << "[REPORT] " << name << " complete\n";
  } catch (const std::exception &error) {
    report_failure_count++;
    std::cerr << "[REPORT-FAIL] " << name << ": " << error.what() << '\n';
  }
}

struct PlantParameters {
  // 실측값이 없는 소형 쿼드의 guessed 파라미터다. 정착시간과 정상상태 수치는
  // 이 값에 의존하지만, 합리적 범위에서 제어 부호와 유계성 판정은 강건하다.
  double arm_length_m = 0.12;
  double ix_kg_m2 = 0.003;
  double iy_kg_m2 = 0.003;
  double iz_kg_m2 = 0.006;
  double thrust_per_us_n = 0.0025;

  double arm_projection_m() const { return arm_length_m / std::sqrt(2.0); }
  double yaw_moment_arm_m() const { return 0.06 * arm_projection_m(); }
};

const PlantParameters kPlantParameters;

struct PlantState {
  double phi = 0.0;
  double theta = 0.0;
  double psi = 0.0;
  double p = 0.0;
  double q = 0.0;
  double r = 0.0;
};

struct Disturbance {
  double x_nm = 0.0;
  double y_nm = 0.0;
  double z_nm = 0.0;
};

struct RunConfig {
  PlantState initial;
  uint32_t ticks = 0;
  float target_roll_deg = 0.0f;
  float target_pitch_deg = 0.0f;
  float target_yaw_deg = 0.0f;
  float ki_roll = 0.005f;
  float ki_pitch = 0.005f;
  float ki_yaw = 0.05f;
  bool inject_roll_sign_fault = false;
  std::function<Disturbance(uint32_t)> disturbance_for_interval;
};

struct Sample {
  uint32_t tick = 0;
  double time_s = 0.0;
  PlantState plant;
  double estimated_roll_deg = 0.0;
  double estimated_pitch_deg = 0.0;
  std::array<int, 4> motors = {1000, 1000, 1000, 1000};
  double i_roll_us = 0.0;
  double i_pitch_us = 0.0;
  double i_yaw_us = 0.0;
  bool yaw_hold = false;
  bool mixer_scaled_now = false;
  bool safety_locked = false;
};

struct RunResult {
  std::vector<Sample> samples;
  bool all_finite = true;
  uint32_t first_nonfinite_tick = std::numeric_limits<uint32_t>::max();
  bool all_motors_in_range = true;
  uint32_t first_bad_motor_tick = std::numeric_limits<uint32_t>::max();
  bool safety_lock_ever = false;
  uint32_t first_safety_lock_tick = std::numeric_limits<uint32_t>::max();
  bool raw_saturated = false;
  uint32_t first_raw_saturation_tick = std::numeric_limits<uint32_t>::max();
  double max_abs_roll_deg = 0.0;
  uint32_t max_abs_roll_tick = 0;
  double max_abs_pitch_deg = 0.0;
  uint32_t max_abs_pitch_tick = 0;
  double max_abs_i_roll_us = 0.0;
  uint32_t max_abs_i_roll_tick = 0;
  double max_abs_i_pitch_us = 0.0;
  uint32_t max_abs_i_pitch_tick = 0;
  double max_abs_i_yaw_us = 0.0;
};

void sendUdpCommandOnce(const std::string &command) {
  wifi_udp_fake::incoming_packet = command;
  arduino_fake::stop_on_task_delay = true;
  bool stopped = false;
  try {
    udp_task(nullptr);
  } catch (const arduino_fake::TaskDelayExit &) {
    stopped = true;
  } catch (...) {
    arduino_fake::stop_on_task_delay = false;
    throw;
  }
  arduino_fake::stop_on_task_delay = false;
  CHECK_MSG(stopped, "udp_task did not stop at the shim delay");
}

int16_t roundedRaw(double value, bool &saturated) {
  if (!std::isfinite(value)) {
    saturated = true;
    return 0;
  }
  const double low = static_cast<double>(std::numeric_limits<int16_t>::min());
  const double high = static_cast<double>(std::numeric_limits<int16_t>::max());
  if (value < low || value > high) saturated = true;
  const double clipped = std::max(low, std::min(high, value));
  return static_cast<int16_t>(std::lround(clipped));
}

int16_t addRaw(int16_t value, int delta, bool &saturated) {
  const int sum = static_cast<int>(value) + delta;
  if (sum < std::numeric_limits<int16_t>::min() ||
      sum > std::numeric_limits<int16_t>::max()) {
    saturated = true;
  }
  return static_cast<int16_t>(std::max(
      static_cast<int>(std::numeric_limits<int16_t>::min()),
      std::min(static_cast<int>(std::numeric_limits<int16_t>::max()), sum)));
}

bool injectImuFromPlant(const PlantState &state, uint32_t tick) {
  bool saturated = false;
  const double p_dps = state.p * kRadToDeg;
  const double q_dps = state.q * kRadToDeg;
  const double r_dps = state.r * kRadToDeg;

  inv_imu_sensor_event_t e1 = {};
  // Physics source: invert the validated accel sensor->body rotation for the
  // roll/pitch gyro too. Do not derive these signs from the firmware gyro map.
  e1.gyro[0] = roundedRaw(-q_dps / GYRO_SCALE, saturated);
  e1.gyro[1] = roundedRaw(p_dps / GYRO_SCALE, saturated);
  // Yaw keeps its separately bench-verified convention.
  e1.gyro[2] = roundedRaw(-r_dps / GYRO_SCALE, saturated);

  const double gbx = -std::sin(state.theta);
  const double gby = std::sin(state.phi) * std::cos(state.theta);
  const double gbz = std::cos(state.phi) * std::cos(state.theta);
  e1.accel[0] = roundedRaw(-gby / ACCEL_SCALE, saturated);
  e1.accel[1] = roundedRaw(gbx / ACCEL_SCALE, saturated);
  e1.accel[2] = roundedRaw(gbz / ACCEL_SCALE, saturated);

  // 정착 후에도 freeze 감시가 동일 프레임으로 오인하지 않게 하는 결정적
  // zero-mean dither다. gyro 1 LSB는 약 0.061 dps, accel은 2 LSB다.
  const int toggle = (tick & 1U) ? -1 : 1;
  e1.gyro[2] = addRaw(e1.gyro[2], toggle, saturated);
  e1.accel[2] = addRaw(e1.accel[2], 2 * toggle, saturated);

  inv_imu_sensor_event_t e2 = {};
  for (int axis = 0; axis < 3; axis++) {
    const int sign = IMU2_SIGN[axis] < 0.0f ? -1 : 1;
    e2.gyro[axis] = addRaw(0, sign * static_cast<int>(e1.gyro[axis]), saturated);
    e2.accel[axis] = addRaw(0, sign * static_cast<int>(e1.accel[axis]), saturated);
  }

  IMU1.next_event = e1;
  IMU2.next_event = e2;
  IMU1.read_status = 0;
  IMU2.read_status = 0;
  return saturated;
}

void resetFirmwareState(const PlantState &initial) {
  arduino_fake::reset();
  arduino_fake::stop_on_task_delay = false;
  wifi_udp_fake::reset();

  connectionEstablished = false;
  laptopIP = IPAddress();
  laptopPort = 0;
  packetBuffer[0] = '\0';

  Kp_Angle_Roll = 6.0f;
  Kp_Angle_Pitch = 6.0f;
  Kp_Angle_Yaw = 3.0f;
  Kp_Rate_Roll = 0.50f;
  Ki_Rate_Roll = 0.005f;
  Kd_Rate_Roll = 0.015f;
  Kp_Rate_Pitch = 0.50f;
  Ki_Rate_Pitch = 0.005f;
  Kd_Rate_Pitch = 0.015f;
  Kp_Rate_Yaw = 1.50f;
  Ki_Rate_Yaw = 0.05f;
  Kd_Rate_Yaw = 0.0f;

  base_throttle = 1000;
  min_throttle = 1050;
  max_throttle = 1300;
  yaw_hold_override = false;
  safety_lock = true;
  calibration_ok = true;

  targetAngleX = 0.0f;
  targetAngleY = 0.0f;
  targetAngleZ = 0.0f;
  angleX = static_cast<float>(initial.phi * kRadToDeg);
  angleY = static_cast<float>(initial.theta * kRadToDeg);
  angleZ = static_cast<float>(initial.psi * kRadToDeg);
  gyroX = static_cast<float>(initial.p * kRadToDeg);
  gyroY = static_cast<float>(initial.q * kRadToDeg);
  gyroZ = static_cast<float>(initial.r * kRadToDeg);
  accX = accY = accZ = 0.0f;
  for (int index = 0; index < 4; index++) motorOut[index] = 1000;
  for (int axis = 0; axis < 3; axis++) tgtRate[axis] = 0.0f;
  pidLoopHz = 0;
  iTermRoll = iTermPitch = iTermYaw = 0.0f;

  for (int axis = 0; axis < 3; axis++) {
    gyro_bias1[axis] = 0.0f;
    gyro_bias2[axis] = 0.0f;
  }
  lastRcMs = 0;
  fault_rc = false;
  fault_imu1 = false;
  fault_imu2 = false;
  fault_disagree = false;
  fault_attitude = false;
  active_imus = 2;
  mixer_scaled = false;
  imu1_frozen_now = false;
  imu2_frozen_now = false;
  imu_disagree_now = false;
  lastRcSeq = 0;
  rcSeqValid = false;
  rcTotalPkts = 0;
  rcDroppedPkts = 0;

  IMU1.next_event = {};
  IMU2.next_event = {};
  IMU1.read_status = 0;
  IMU2.read_status = 0;
  (void)injectImuFromPlant(initial, 0);
}

Disturbance disturbanceAt(const RunConfig &config, uint32_t interval) {
  if (!config.disturbance_for_interval) return {};
  return config.disturbance_for_interval(interval);
}

void integratePlant(PlantState &state, const Disturbance &disturbance,
                    bool inject_roll_sign_fault) {
  std::array<double, 4> applied = {
      static_cast<double>(motorOut[0]), static_cast<double>(motorOut[1]),
      static_cast<double>(motorOut[2]), static_cast<double>(motorOut[3])};

  if (inject_roll_sign_fault) {
    // 뮤테이션은 mixer roll 축의 부호 하나(roll allocation column 전체)를
    // R -> -R로 뒤집는다. 모터 하나의 계수만 바꾸면 authority가 절반으로
    // 줄 뿐 부호가 유지되어 발산 결함이 아니다. 아래 기체 레이아웃/r×F
    // 토크식은 그대로 두므로 검증이 tautology가 되지 않는다.
    const double roll_mode =
        (applied[0] - applied[1] - applied[2] + applied[3]) * 0.25;
    applied[0] -= 2.0 * roll_mode;
    applied[1] += 2.0 * roll_mode;
    applied[2] += 2.0 * roll_mode;
    applied[3] -= 2.0 * roll_mode;
    for (double &motor : applied) {
      motor = std::max(static_cast<double>(min_throttle),
                       std::min(static_cast<double>(max_throttle), motor));
    }
  }

  std::array<double, 4> thrust = {};
  for (std::size_t index = 0; index < thrust.size(); index++) {
    thrust[index] = kPlantParameters.thrust_per_us_n *
                    std::max(0.0, applied[index] - 1000.0);
  }

  const double a = kPlantParameters.arm_projection_m();
  const double tau_x = a * (thrust[0] - thrust[1] - thrust[2] + thrust[3]);
  const double tau_y = a * (-thrust[0] + thrust[1] - thrust[2] + thrust[3]);
  // CW(+)/CCW(-) 반작용 관례다. 실제 yaw 부호는 전원 벤치에서 확정한다.
  const double tau_z = kPlantParameters.yaw_moment_arm_m() *
                       (thrust[0] + thrust[1] - thrust[2] - thrust[3]);

  const double old_p = state.p;
  const double old_q = state.q;
  const double old_r = state.r;
  state.phi += old_p * kDt;
  state.theta += old_q * kDt;
  state.psi += old_r * kDt;
  state.p += (tau_x + disturbance.x_nm) / kPlantParameters.ix_kg_m2 * kDt;
  state.q += (tau_y + disturbance.y_nm) / kPlantParameters.iy_kg_m2 * kDt;
  state.r += (tau_z + disturbance.z_nm) / kPlantParameters.iz_kg_m2 * kDt;
}

bool finiteState(const PlantState &state) {
  return std::isfinite(state.phi) && std::isfinite(state.theta) &&
         std::isfinite(state.psi) && std::isfinite(state.p) &&
         std::isfinite(state.q) && std::isfinite(state.r);
}

void appendSample(RunResult &result, uint32_t tick, const PlantState &state) {
  Sample sample;
  sample.tick = tick;
  sample.time_s = tick * kDt;
  sample.plant = state;
  sample.estimated_roll_deg = angleX;
  sample.estimated_pitch_deg = angleY;
  sample.i_roll_us = iTermRoll;
  sample.i_pitch_us = iTermPitch;
  sample.i_yaw_us = iTermYaw;
  sample.yaw_hold = yaw_hold_now;
  sample.mixer_scaled_now = mixer_scaled;
  sample.safety_locked = safety_lock;
  for (int index = 0; index < 4; index++) sample.motors[index] = motorOut[index];
  result.samples.push_back(sample);

  const bool firmware_finite =
      std::isfinite(static_cast<double>(angleX)) &&
      std::isfinite(static_cast<double>(angleY)) &&
      std::isfinite(static_cast<double>(angleZ)) &&
      std::isfinite(static_cast<double>(gyroX)) &&
      std::isfinite(static_cast<double>(gyroY)) &&
      std::isfinite(static_cast<double>(gyroZ)) &&
      std::isfinite(static_cast<double>(iTermRoll)) &&
      std::isfinite(static_cast<double>(iTermPitch)) &&
      std::isfinite(static_cast<double>(iTermYaw));
  if (result.all_finite && (!finiteState(state) || !firmware_finite)) {
    result.all_finite = false;
    result.first_nonfinite_tick = tick;
  }

  for (int motor : sample.motors) {
    if (result.all_motors_in_range && (motor < 1000 || motor > 2000)) {
      result.all_motors_in_range = false;
      result.first_bad_motor_tick = tick;
    }
  }
  if (!result.safety_lock_ever && sample.safety_locked) {
    result.safety_lock_ever = true;
    result.first_safety_lock_tick = tick;
  }

  const double abs_roll_deg = std::fabs(state.phi * kRadToDeg);
  const double abs_pitch_deg = std::fabs(state.theta * kRadToDeg);
  if (abs_roll_deg > result.max_abs_roll_deg) {
    result.max_abs_roll_deg = abs_roll_deg;
    result.max_abs_roll_tick = tick;
  }
  if (abs_pitch_deg > result.max_abs_pitch_deg) {
    result.max_abs_pitch_deg = abs_pitch_deg;
    result.max_abs_pitch_tick = tick;
  }
  const double abs_i_roll_us = std::fabs(sample.i_roll_us);
  const double abs_i_pitch_us = std::fabs(sample.i_pitch_us);
  const double abs_i_yaw_us = std::fabs(sample.i_yaw_us);
  if (abs_i_roll_us > result.max_abs_i_roll_us) {
    result.max_abs_i_roll_us = abs_i_roll_us;
    result.max_abs_i_roll_tick = tick;
  }
  if (abs_i_pitch_us > result.max_abs_i_pitch_us) {
    result.max_abs_i_pitch_us = abs_i_pitch_us;
    result.max_abs_i_pitch_tick = tick;
  }
  if (abs_i_yaw_us > result.max_abs_i_yaw_us) {
    result.max_abs_i_yaw_us = abs_i_yaw_us;
  }
}

RunResult runSil(const RunConfig &config) {
  CHECK_MSG(config.ticks > 0, "tick limit 0 would make pid_task unbounded");
  PlantState state = config.initial;
  resetFirmwareState(state);

  sendUdpCommandOnce("start");
  CHECK_MSG(!safety_lock, "start command was refused");
  sendUdpCommandOnce("th 1150");
  CHECK_EQ(base_throttle, 1150);
  CHECK_EQ(min_throttle, 1050);
  CHECK_EQ(max_throttle, 1300);

  targetAngleX = config.target_roll_deg;
  targetAngleY = config.target_pitch_deg;
  targetAngleZ = config.target_yaw_deg;
  angleX = static_cast<float>(state.phi * kRadToDeg);
  angleY = static_cast<float>(state.theta * kRadToDeg);
  angleZ = static_cast<float>(state.psi * kRadToDeg);
  Ki_Rate_Roll = config.ki_roll;
  Ki_Rate_Pitch = config.ki_pitch;
  Ki_Rate_Yaw = config.ki_yaw;

  RunResult result;
  result.samples.reserve(static_cast<std::size_t>(config.ticks) + 1U);
  arduino_fake::pre_tick_hook = [&](uint32_t tick) {
    if (tick > 0) {
      integratePlant(state, disturbanceAt(config, tick - 1U),
                     config.inject_roll_sign_fault);
    }
    arduino_fake::millis_value += 1U;
    arduino_fake::micros_value += 1000U;
    lastRcMs = millis();
    const bool raw_saturated_now = injectImuFromPlant(state, tick);
    if (raw_saturated_now && !result.raw_saturated) {
      result.first_raw_saturation_tick = tick;
    }
    result.raw_saturated = raw_saturated_now || result.raw_saturated;
    appendSample(result, tick, state);
  };
  arduino_fake::tick_limit = config.ticks;

  bool stopped = false;
  try {
    pid_task(nullptr);
  } catch (const arduino_fake::TaskDelayExit &) {
    stopped = true;
  } catch (...) {
    arduino_fake::pre_tick_hook = nullptr;
    arduino_fake::tick_limit = 0;
    throw;
  }
  arduino_fake::pre_tick_hook = nullptr;
  arduino_fake::tick_limit = 0;

  CHECK_MSG(stopped, "pid_task did not stop at tick_limit");
  CHECK_EQ(result.samples.size(), static_cast<std::size_t>(config.ticks) + 1U);
  return result;
}

struct EstimatorTrackingResult {
  double estimate_at_ramp_end_deg = 0.0;
  double estimate_at_deadline_deg = 0.0;
  uint32_t first_90_percent_tick = std::numeric_limits<uint32_t>::max();
  bool raw_saturated = false;
};

EstimatorTrackingResult runPrescribedEstimatorRamp(bool roll) {
  constexpr uint32_t kRampTicks = 1000;
  constexpr uint32_t kDeadlineTick = kRampTicks + 150;
  constexpr double kCommandDeg = 30.0;
  constexpr double kRampRateDegPerSec =
      kCommandDeg / (kRampTicks * kDt);
  constexpr double kNinetyPercentDeg = 0.9 * kCommandDeg;

  PlantState initial;
  resetFirmwareState(initial);

  EstimatorTrackingResult result;
  arduino_fake::pre_tick_hook = [&](uint32_t tick) {
    const double estimate_deg = roll ? angleX : angleY;
    if (tick == kRampTicks) {
      result.estimate_at_ramp_end_deg = estimate_deg;
    }
    if (tick == kDeadlineTick) {
      result.estimate_at_deadline_deg = estimate_deg;
    }
    if (result.first_90_percent_tick ==
            std::numeric_limits<uint32_t>::max() &&
        estimate_deg >= kNinetyPercentDeg) {
      result.first_90_percent_tick = tick;
    }

    arduino_fake::millis_value += 1U;
    arduino_fake::micros_value += 1000U;
    lastRcMs = millis();

    const double ramp_fraction =
        std::min(1.0, static_cast<double>(tick) / kRampTicks);
    PlantState prescribed;
    if (roll) {
      prescribed.phi = ramp_fraction * kCommandDeg * kDegToRad;
      prescribed.p =
          tick < kRampTicks ? kRampRateDegPerSec * kDegToRad : 0.0;
    } else {
      prescribed.theta = ramp_fraction * kCommandDeg * kDegToRad;
      prescribed.q =
          tick < kRampTicks ? kRampRateDegPerSec * kDegToRad : 0.0;
    }
    result.raw_saturated =
        injectImuFromPlant(prescribed, tick) || result.raw_saturated;
  };
  arduino_fake::tick_limit = kDeadlineTick;

  bool stopped = false;
  try {
    pid_task(nullptr);
  } catch (const arduino_fake::TaskDelayExit &) {
    stopped = true;
  } catch (...) {
    arduino_fake::pre_tick_hook = nullptr;
    arduino_fake::tick_limit = 0;
    throw;
  }
  arduino_fake::pre_tick_hook = nullptr;
  arduino_fake::tick_limit = 0;

  CHECK_MSG(stopped, "pid_task did not stop at estimator ramp deadline");
  return result;
}

double meanAbsTailDeg(const RunResult &result, bool roll, std::size_t count) {
  CHECK_MSG(!result.samples.empty(), "trajectory is empty");
  count = std::min(count, result.samples.size());
  const std::size_t begin = result.samples.size() - count;
  double sum = 0.0;
  for (std::size_t index = begin; index < result.samples.size(); index++) {
    const double angle = roll ? result.samples[index].plant.phi
                              : result.samples[index].plant.theta;
    sum += std::fabs(angle * kRadToDeg);
  }
  return sum / static_cast<double>(count);
}

double settlingTime90(const RunResult &result, bool roll,
                      double initial_abs_deg) {
  const double band_deg = 0.1 * initial_abs_deg;
  std::size_t last_outside = 0;
  bool was_outside = false;
  for (std::size_t index = 0; index < result.samples.size(); index++) {
    const double angle_deg = (roll ? result.samples[index].plant.phi
                                   : result.samples[index].plant.theta) * kRadToDeg;
    if (!std::isfinite(angle_deg)) {
      return std::numeric_limits<double>::infinity();
    }
    if (std::fabs(angle_deg) > band_deg) {
      was_outside = true;
      last_outside = index;
    }
  }
  if (!was_outside) return 0.0;
  if (last_outside + 1U >= result.samples.size()) {
    return std::numeric_limits<double>::infinity();
  }
  return result.samples[last_outside + 1U].time_s;
}

const Sample &sampleAtTick(const RunResult &result, uint32_t tick) {
  CHECK_MSG(tick < result.samples.size(), "requested sample is outside trajectory");
  return result.samples[tick];
}

std::string tickDetail(const char *label, uint32_t tick) {
  std::ostringstream detail;
  detail << label << " at tick " << tick;
  return detail.str();
}

std::string metricTickDetail(const char *label, double value,
                             uint32_t tick, double limit) {
  std::ostringstream detail;
  detail << label << "=" << value << " at tick " << tick
         << ", limit=" << limit;
  return detail.str();
}

std::string transitionDetail(const char *label, double from_value,
                             uint32_t from_tick, double to_value,
                             uint32_t to_tick) {
  std::ostringstream detail;
  detail << label << ": " << from_value << " at tick " << from_tick
         << " -> " << to_value << " at tick " << to_tick;
  return detail.str();
}

RunConfig constantRollDisturbance(float ki_roll, double torque_nm,
                                  uint32_t ticks) {
  RunConfig config;
  config.ticks = ticks;
  config.ki_roll = ki_roll;
  config.disturbance_for_interval = [torque_nm](uint32_t) {
    return Disturbance{torque_nm, 0.0, 0.0};
  };
  return config;
}

RunConfig constantYawDisturbance(float ki_yaw, double torque_nm,
                                 uint32_t ticks) {
  RunConfig config;
  config.ticks = ticks;
  config.ki_yaw = ki_yaw;
  config.disturbance_for_interval = [torque_nm](uint32_t) {
    return Disturbance{0.0, 0.0, torque_nm};
  };
  return config;
}

double tailMeanAbsYawRateDps(const RunResult &result, std::size_t tail) {
  const std::size_t n = result.samples.size();
  const std::size_t start = n > tail ? n - tail : 0;
  double sum = 0.0;
  for (std::size_t index = start; index < n; index++) {
    sum += std::fabs(result.samples[index].plant.r) * kRadToDeg;
  }
  return sum / static_cast<double>(n - start);
}

}  // namespace

int main() {
  static_assert(sizeof(long) == 4, "SIL은 ESP32와 같은 32비트 long이 필요하다");
  std::cout << std::fixed << std::setprecision(4);
  std::cout << "[SIL] SIL_INJECT_SIGN_FAULT="
            << (kInjectRollSignFault ? "ON" : "OFF") << '\n';

  runCase("shim: tick hook order, limit, and reset", [] {
    arduino_fake::reset();
    std::vector<uint32_t> observed;
    arduino_fake::pre_tick_hook = [&](uint32_t tick) { observed.push_back(tick); };
    arduino_fake::tick_limit = 2;

    TickType_t wake = 0;
    vTaskDelayUntil(&wake, 1);
    vTaskDelayUntil(&wake, 1);
    bool stopped = false;
    try {
      vTaskDelayUntil(&wake, 1);
    } catch (const arduino_fake::TaskDelayExit &) {
      stopped = true;
    }

    CHECK(stopped);
    CHECK_EQ(observed.size(), static_cast<std::size_t>(3));
    CHECK_EQ(observed[0], 0U);
    CHECK_EQ(observed[1], 1U);
    CHECK_EQ(observed[2], 2U);
    CHECK_EQ(arduino_fake::tick_index, 3U);

    arduino_fake::reset();
    CHECK(!arduino_fake::pre_tick_hook);
    CHECK_EQ(arduino_fake::tick_index, 0U);
    CHECK_EQ(arduino_fake::tick_limit, 0U);
  });

  runCase("helper: non-finite trajectory cannot settle", [] {
    RunResult result;
    Sample initial;
    initial.plant.phi = 8.0 * kDegToRad;
    result.samples.push_back(initial);
    Sample invalid;
    invalid.tick = 1;
    invalid.time_s = kDt;
    invalid.plant.phi = std::numeric_limits<double>::quiet_NaN();
    result.samples.push_back(invalid);
    CHECK(!std::isfinite(settlingTime90(result, true, 8.0)));
  });

  runCase("gyro transform: prescribed +30deg roll tracks by 150ms", [] {
    constexpr uint32_t kDeadlineTick = 1150;
    const EstimatorTrackingResult result =
        runPrescribedEstimatorRamp(true);
    std::cout << "[SIL] gyro-axis roll estimate@1.000s="
              << result.estimate_at_ramp_end_deg
              << "deg estimate@1.150s=" << result.estimate_at_deadline_deg
              << "deg first>=27deg_tick=" << result.first_90_percent_tick
              << '\n';

    CHECK_MSG(!result.raw_saturated,
              "roll prescribed-motion synthetic IMU raw saturated");
    CHECK_MSG(result.estimate_at_deadline_deg > 0.0,
              "roll estimate has the wrong sign");
    CHECK_MSG(result.estimate_at_deadline_deg >= 27.0,
              "roll estimate did not reach 90% of +30deg");
    CHECK_MSG(result.first_90_percent_tick <= kDeadlineTick,
              "roll estimate did not reach 90% by 150ms after the ramp");
  });

  runCase("gyro transform: prescribed +30deg pitch tracks by 150ms", [] {
    constexpr uint32_t kDeadlineTick = 1150;
    const EstimatorTrackingResult result =
        runPrescribedEstimatorRamp(false);
    std::cout << "[SIL] gyro-axis pitch estimate@1.000s="
              << result.estimate_at_ramp_end_deg
              << "deg estimate@1.150s=" << result.estimate_at_deadline_deg
              << "deg first>=27deg_tick=" << result.first_90_percent_tick
              << '\n';

    CHECK_MSG(!result.raw_saturated,
              "pitch prescribed-motion synthetic IMU raw saturated");
    CHECK_MSG(result.estimate_at_deadline_deg > 0.0,
              "pitch estimate has the wrong sign");
    CHECK_MSG(result.estimate_at_deadline_deg >= 27.0,
              "pitch estimate did not reach 90% of +30deg");
    CHECK_MSG(result.first_90_percent_tick <= kDeadlineTick,
              "pitch estimate did not reach 90% by 150ms after the ramp");
  });

  runCase("S1: roll/pitch attitude hold converges", [] {
    RunConfig config;
    config.initial.phi = 8.0 * kDegToRad;
    config.initial.theta = -6.0 * kDegToRad;
    config.ticks = 3000;
    config.inject_roll_sign_fault = kInjectRollSignFault;
    const RunResult result = runSil(config);
    const double roll_tail = meanAbsTailDeg(result, true, 500);
    const double pitch_tail = meanAbsTailDeg(result, false, 500);
    std::cout << "[SIL] S1 max|phi|=" << result.max_abs_roll_deg
              << "deg@" << result.max_abs_roll_tick
              << " max|theta|=" << result.max_abs_pitch_deg
              << "deg@" << result.max_abs_pitch_tick
              << " tail500_mean|phi|=" << roll_tail
              << "deg tail500_mean|theta|=" << pitch_tail << "deg\n";

    CHECK_MSG(result.max_abs_roll_deg < 12.0,
              metricTickDetail("max|phi| deg", result.max_abs_roll_deg,
                               result.max_abs_roll_tick, 12.0));
    CHECK_MSG(result.max_abs_pitch_deg < 12.0,
              metricTickDetail("max|theta| deg", result.max_abs_pitch_deg,
                               result.max_abs_pitch_tick, 12.0));
    CHECK_MSG(roll_tail < 1.5,
              metricTickDetail("tail500 mean|phi| deg", roll_tail,
                               result.samples.back().tick, 1.5));
    CHECK_MSG(pitch_tail < 1.5,
              metricTickDetail("tail500 mean|theta| deg", pitch_tail,
                               result.samples.back().tick, 1.5));
    CHECK_MSG(result.all_motors_in_range,
              tickDetail("motor output left [1000,2000]",
                         result.first_bad_motor_tick));
    CHECK_MSG(result.all_finite,
              tickDetail("non-finite SIL state", result.first_nonfinite_tick));
    CHECK_MSG(!result.raw_saturated,
              tickDetail("synthetic IMU raw saturated",
                         result.first_raw_saturation_tick));
    CHECK_MSG(!result.samples.back().safety_locked,
              tickDetail("safety lock active", result.samples.back().tick));
  });

  runCase("S2: roll/pitch settling symmetry", [] {
    RunConfig roll_config;
    roll_config.initial.phi = 8.0 * kDegToRad;
    roll_config.ticks = 3000;
    const RunResult roll_result = runSil(roll_config);

    RunConfig pitch_config;
    pitch_config.initial.theta = 8.0 * kDegToRad;
    pitch_config.ticks = 3000;
    const RunResult pitch_result = runSil(pitch_config);

    CHECK_MSG(roll_result.all_finite,
              tickDetail("non-finite roll symmetry state",
                         roll_result.first_nonfinite_tick));
    CHECK_MSG(pitch_result.all_finite,
              tickDetail("non-finite pitch symmetry state",
                         pitch_result.first_nonfinite_tick));
    CHECK_MSG(!roll_result.raw_saturated,
              tickDetail("roll symmetry IMU raw saturated",
                         roll_result.first_raw_saturation_tick));
    CHECK_MSG(!pitch_result.raw_saturated,
              tickDetail("pitch symmetry IMU raw saturated",
                         pitch_result.first_raw_saturation_tick));

    const double roll_settle = settlingTime90(roll_result, true, 8.0);
    const double pitch_settle = settlingTime90(pitch_result, false, 8.0);
    const double ratio = roll_settle / pitch_settle;
    std::cout << "[SIL] S2 settle90 roll=" << roll_settle
              << "s pitch=" << pitch_settle << "s ratio=" << ratio << '\n';

    CHECK_MSG(std::isfinite(roll_settle), "roll did not settle inside 3 s");
    CHECK_MSG(std::isfinite(pitch_settle), "pitch did not settle inside 3 s");
    CHECK_GE(ratio, 0.6);
    CHECK_LE(ratio, 1.6);
  });

  runCase("S3: anti-windup clamps and recovers after saturation", [] {
    constexpr uint32_t kPreloadEnd = 8000;
    constexpr uint32_t kSaturationEnd = 8040;
    constexpr uint32_t kRunEnd = 13000;
    RunConfig config;
    config.ticks = kRunEnd;
    // 요구된 Ki sweep의 0.5 값을 사용해 native 실행 시간 안에 clamp 경로를
    // 충분히 자극한다. 펌웨어의 ±50 us 한계 자체는 바꾸지 않는다.
    config.ki_roll = 0.5f;
    config.ki_pitch = 0.5f;
    config.disturbance_for_interval = [](uint32_t tick) {
      if (tick < 2000U) {
        return Disturbance{0.060 * static_cast<double>(tick) / 2000.0,
                           0.0, 0.0};
      }
      if (tick < kPreloadEnd) return Disturbance{0.060, 0.0, 0.0};
      if (tick < kSaturationEnd) return Disturbance{0.300, 0.0, 0.0};
      return Disturbance{};
    };
    const RunResult result = runSil(config);

    uint32_t scaled_samples = 0;
    uint32_t max_roll_tick = 0;
    double max_roll_deg = 0.0;
    for (const Sample &sample : result.samples) {
      if (sample.tick >= kPreloadEnd && sample.tick <= kSaturationEnd &&
          sample.mixer_scaled_now) {
        scaled_samples++;
      }
      const double roll_deg = std::fabs(sample.plant.phi * kRadToDeg);
      if (roll_deg > max_roll_deg) {
        max_roll_deg = roll_deg;
        max_roll_tick = sample.tick;
      }
    }
    const double removal_i = sampleAtTick(result, kSaturationEnd).i_roll_us;
    const double final_i = result.samples.back().i_roll_us;

    // 별도 짧은 런은 clamp에 닿기 전에 포화시켜 !mix.scaled 조건 자체를
    // 검증한다. 연속 scaled tick에서 iTerm이 한 비트도 움직이면 실패한다.
    RunConfig gate_config;
    gate_config.ticks = 1500;
    gate_config.ki_roll = 0.5f;
    gate_config.ki_pitch = 0.5f;
    gate_config.disturbance_for_interval = [](uint32_t tick) {
      return tick < 60U ? Disturbance{0.300, 0.0, 0.0} : Disturbance{};
    };
    const RunResult gate_result = runSil(gate_config);
    uint32_t gate_hold_pairs = 0;
    double gate_max_i_delta = 0.0;
    uint32_t gate_max_i_delta_tick = 0;
    for (std::size_t index = 1; index < gate_result.samples.size(); index++) {
      const Sample &previous = gate_result.samples[index - 1U];
      const Sample &current = gate_result.samples[index];
      const bool below_clamp =
          std::fabs(previous.i_roll_us) < I_TERM_MAX_US - 5.0 &&
          std::fabs(current.i_roll_us) < I_TERM_MAX_US - 5.0;
      if (previous.mixer_scaled_now && current.mixer_scaled_now && below_clamp) {
        gate_hold_pairs++;
        const double delta =
            std::fabs(current.i_roll_us - previous.i_roll_us);
        if (delta > gate_max_i_delta) {
          gate_max_i_delta = delta;
          gate_max_i_delta_tick = current.tick;
        }
      }
    }
    std::cout << "[SIL] S3 max|iRoll|=" << result.max_abs_i_roll_us
              << "us@" << result.max_abs_i_roll_tick
              << " max|iPitch|=" << result.max_abs_i_pitch_us
              << "us@" << result.max_abs_i_pitch_tick
              << " scaled_samples=" << scaled_samples
              << " removal_iRoll=" << removal_i
              << "us final_iRoll=" << final_i
              << "us max|phi|=" << result.max_abs_roll_deg
              << "deg@" << max_roll_tick
              << " preload_phi="
              << sampleAtTick(result, kPreloadEnd).plant.phi * kRadToDeg
              << "deg gate_hold_pairs=" << gate_hold_pairs
              << " gate_max_i_delta=" << gate_max_i_delta
              << "us@" << gate_max_i_delta_tick << "\n";

    CHECK_MSG(result.max_abs_i_roll_us <= I_TERM_MAX_US + 0.001,
              metricTickDetail("max|iTermRoll| us", result.max_abs_i_roll_us,
                               result.max_abs_i_roll_tick,
                               I_TERM_MAX_US + 0.001));
    CHECK_MSG(result.max_abs_i_pitch_us <= I_TERM_MAX_US + 0.001,
              metricTickDetail("max|iTermPitch| us", result.max_abs_i_pitch_us,
                               result.max_abs_i_pitch_tick,
                               I_TERM_MAX_US + 0.001));
    CHECK_MSG(result.max_abs_i_roll_us >= I_TERM_MAX_US - 1.0,
              metricTickDetail("max|iTermRoll| us", result.max_abs_i_roll_us,
                               result.max_abs_i_roll_tick,
                               I_TERM_MAX_US - 1.0));
    CHECK_MSG(std::fabs(removal_i) >= I_TERM_MAX_US - 1.0,
              metricTickDetail("removal |iTermRoll| us", std::fabs(removal_i),
                               kSaturationEnd, I_TERM_MAX_US - 1.0));
    CHECK_MSG(scaled_samples > 0, "disturbance did not exercise mixer saturation");
    CHECK_MSG(!result.safety_lock_ever,
              tickDetail("S3 invalidated by safety lock",
                         result.first_safety_lock_tick));
    CHECK_MSG(result.all_finite,
              tickDetail("non-finite S3 state", result.first_nonfinite_tick));
    CHECK_MSG(!result.raw_saturated,
              tickDetail("S3 synthetic IMU raw saturated",
                         result.first_raw_saturation_tick));
    CHECK_MSG(std::fabs(final_i) + 1.0 < std::fabs(removal_i),
              transitionDetail("iTermRoll recovery", removal_i,
                               kSaturationEnd, final_i,
                               result.samples.back().tick));
    CHECK_MSG(gate_result.all_finite,
              tickDetail("non-finite saturation-gate probe state",
                         gate_result.first_nonfinite_tick));
    CHECK_MSG(!gate_result.raw_saturated,
              tickDetail("saturation-gate probe IMU raw saturated",
                         gate_result.first_raw_saturation_tick));
    CHECK_MSG(!gate_result.safety_lock_ever,
              tickDetail("saturation-gate probe hit safety lock",
                         gate_result.first_safety_lock_tick));
    CHECK_MSG(gate_hold_pairs >= 5,
              "saturation-gate probe lacked a contiguous below-clamp interval");
    CHECK_MSG(gate_max_i_delta <= 1e-7,
              metricTickDetail("scaled iTermRoll delta us", gate_max_i_delta,
                               gate_max_i_delta_tick, 1e-7));
  });

  runReport("S4 integrator authority (numbers only)", [] {
    const double roll_torque_per_us =
        4.0 * kPlantParameters.arm_projection_m() *
        kPlantParameters.thrust_per_us_n;
    const double disturbance_nm = roll_torque_per_us * 0.50 * 6.0 * 2.5;

    const RunResult p_only = runSil(
        constantRollDisturbance(0.0f, disturbance_nm, 10000));
    const double p_only_ss = meanAbsTailDeg(p_only, true, 500);
    std::cout << "[SIL] S4 P-only Ki=0.000 10s |phi_ss|="
              << p_only_ss << "deg iTermRoll="
              << p_only.samples.back().i_roll_us
              << "us raw_saturated=" << p_only.raw_saturated << "\n";

    const RunResult nominal = runSil(
        constantRollDisturbance(0.005f, disturbance_nm, 10000));
    std::cout << "[SIL] S4 Ki=0.005 |phi|(1s,2s,5s,10s)="
              << std::fabs(sampleAtTick(nominal, 1000).plant.phi * kRadToDeg) << ","
              << std::fabs(sampleAtTick(nominal, 2000).plant.phi * kRadToDeg) << ","
              << std::fabs(sampleAtTick(nominal, 5000).plant.phi * kRadToDeg) << ","
              << std::fabs(sampleAtTick(nominal, 10000).plant.phi * kRadToDeg)
              << "deg final_iTermRoll=" << nominal.samples.back().i_roll_us
              << "us authority="
              << 100.0 * std::fabs(nominal.samples.back().i_roll_us) /
                            I_TERM_MAX_US
              << "% of +/-50us raw_saturated=" << nominal.raw_saturated << "\n";

    const auto print_sweep = [](double ki, const RunResult &result) {
      std::cout << "[SIL] S4 sweep Ki=" << ki
                << " 10s |phi|="
                << std::fabs(result.samples.back().plant.phi * kRadToDeg)
                << "deg iTermRoll=" << result.samples.back().i_roll_us
                << "us raw_saturated=" << result.raw_saturated << "\n";
    };
    print_sweep(0.005, nominal);
    const RunResult ki_005 = runSil(
        constantRollDisturbance(0.05f, disturbance_nm, 10000));
    print_sweep(0.05, ki_005);
    const RunResult ki_05 = runSil(
        constantRollDisturbance(0.5f, disturbance_nm, 10000));
    print_sweep(0.5, ki_05);

    std::cout << "[SIL] S4 note: guessed plant parameters make settling/steady-state "
                 "numbers order-of-magnitude; only the conclusion that Ki=0.005 is "
                 "effectively inactive on a 10s hover timescale is used.\n";
  });

  runReport("S5 yaw damping (numbers only)", [] {
    RunConfig config;
    config.initial.r = 60.0 * kDegToRad;
    config.ticks = 2000;
    const RunResult result = runSil(config);
    std::cout << "[SIL] S5 signed r(dps) t=0,0.5,1.0,1.5,2.0s: "
              << sampleAtTick(result, 0).plant.r * kRadToDeg << ","
              << sampleAtTick(result, 500).plant.r * kRadToDeg << ","
              << sampleAtTick(result, 1000).plant.r * kRadToDeg << ","
              << sampleAtTick(result, 1500).plant.r * kRadToDeg << ","
              << sampleAtTick(result, 2000).plant.r * kRadToDeg
              << " raw_saturated=" << result.raw_saturated << '\n';
    std::cout << "[SIL] S5 note: yaw reaction-torque sign is convention-dependent; "
                 "confirm prop direction on a powered bench before asserting damping.\n";
  });

  runCase("S6 yaw 적분 해금: hold가 아닌 구간에서도 적분기가 누적된다", [] {
    // SIL yaw 플랜트의 반작용 토크 부호는 미해결이다(S5 참조: runReport로만
    // 남기고 단언하지 않는다). 따라서 폐루프 정상상태 각속도로는 단언할 수
    // 없다. 이 케이스는 코드 변경 자체 — "hold가 아니어도 iTermYaw가
    // 누적된다" — 만 검증하며 플랜트 부호와 무관하다.
    const double yaw_torque_per_us =
        4.0 * kPlantParameters.yaw_moment_arm_m() *
        kPlantParameters.thrust_per_us_n;
    const double disturbance_nm = yaw_torque_per_us * 1.50 * 20.0;

    const RunResult result =
        runSil(constantYawDisturbance(0.05f, disturbance_nm, 3000));

    std::size_t rate_mode_samples = 0;
    double max_i_yaw_rate_mode = 0.0;
    for (const Sample &sample : result.samples) {
      if (sample.yaw_hold) continue;
      rate_mode_samples++;
      max_i_yaw_rate_mode =
          std::max(max_i_yaw_rate_mode, std::fabs(sample.i_yaw_us));
    }

    // 판별력 확인: hold가 아닌 구간이 실제로 있어야 이 테스트가 의미를 갖는다.
    CHECK_MSG(rate_mode_samples > 100,
              "hold가 아닌 구간이 거의 없어 해금 여부를 가릴 수 없다");
    // 해금 전에는 hold가 아니면 iTermYaw가 0으로 묶였다. 여기서 red/green이 갈린다.
    CHECK_MSG(max_i_yaw_rate_mode > 0.0,
              "rate 모드에서 yaw 적분기가 전혀 누적되지 않았다 (해금 실패)");
    CHECK_LE(result.max_abs_i_yaw_us, static_cast<double>(I_TERM_MAX_US));
    std::cout << "[SIL] S6 rate-mode samples=" << rate_mode_samples
              << " max|iTermYaw| in rate mode=" << max_i_yaw_rate_mode
              << "us, overall max=" << result.max_abs_i_yaw_us << "us\n";
  });

  runReport("S6b yaw closed-loop rate (numbers only)", [] {
    // 플랜트 yaw 부호가 확정되기 전까지 수치만 남긴다. S5와 같은 취급이다.
    const double yaw_torque_per_us =
        4.0 * kPlantParameters.yaw_moment_arm_m() *
        kPlantParameters.thrust_per_us_n;
    const double disturbance_nm = yaw_torque_per_us * 1.50 * 20.0;
    const RunResult p_only =
        runSil(constantYawDisturbance(0.0f, disturbance_nm, 3000));
    const RunResult with_i =
        runSil(constantYawDisturbance(0.05f, disturbance_nm, 3000));
    std::cout << "[SIL] S6b tail |r| P-only="
              << tailMeanAbsYawRateDps(p_only, 500) << "dps Ki=0.05="
              << tailMeanAbsYawRateDps(with_i, 500)
              << "dps -- 플랜트 yaw 부호 미해결, 벤치 Stage D에서 확정\n";
  });

  std::cout << "\n" << (test_count - failure_count) << "/" << test_count
            << " hard native attitude SIL cases passed; "
            << (report_count - report_failure_count) << "/" << report_count
            << " report sections completed\n";
  return failure_count + report_failure_count;
}
