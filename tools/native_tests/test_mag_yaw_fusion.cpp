#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include "dual_imu_cascade_pwm.ino"

namespace {

constexpr float kDt = 0.001f;
constexpr float kDegToRad = PI / 180.0f;
constexpr float kRadToDeg = 180.0f / PI;

int test_count = 0;
int failure_count = 0;

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

[[maybe_unused]] void checkNear(float actual, float expected, float tolerance,
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

[[maybe_unused]] void runCase(
    const std::string &name, const std::function<void()> &body) {
  test_count++;
  try {
    body();
    std::cout << "[PASS] " << name << '\n';
  } catch (const std::exception &error) {
    failure_count++;
    std::cerr << "[FAIL] " << name << ": " << error.what() << '\n';
  }
}

int16_t rawValue(float value) {
  const float clipped = std::max(
      static_cast<float>(std::numeric_limits<int16_t>::min()),
      std::min(static_cast<float>(std::numeric_limits<int16_t>::max()), value));
  return static_cast<int16_t>(std::lround(clipped));
}

void resetCommonFirmwareState(float roll_deg, float pitch_deg, float yaw_deg) {
  arduino_fake::reset();
  wifi_udp_fake::reset();

  Kp_Angle_Roll = 6.0f;
  Kp_Angle_Pitch = 6.0f;
  Kp_Angle_Yaw = 3.0f;
  Kp_Rate_Roll = 0.50f;
  Ki_Rate_Roll = 0.05f;
  Kd_Rate_Roll = 0.015f;
  Kp_Rate_Pitch = 0.50f;
  Ki_Rate_Pitch = 0.05f;
  Kd_Rate_Pitch = 0.015f;
  Kp_Rate_Yaw = 1.50f;
  Ki_Rate_Yaw = 0.05f;
  Kd_Rate_Yaw = 0.0f;

  base_throttle = 1150;
  min_throttle = 1050;
  max_throttle = 1300;
  yaw_enabled = false;
  safety_lock = false;
  calibration_ok = true;

  targetAngleX = roll_deg;
  targetAngleY = pitch_deg;
  targetAngleZ = yaw_deg;
  angleX = roll_deg;
  angleY = pitch_deg;
  angleZ = yaw_deg;
  gyroX = 0.0f;
  gyroY = 0.0f;
  gyroZ = 0.0f;
  accX = 0.0f;
  accY = 0.0f;
  accZ = 1.0f;
  motorOut[0] = 1000;
  motorOut[1] = 1000;
  motorOut[2] = 1000;
  motorOut[3] = 1000;
  tgtRate[0] = 0.0f;
  tgtRate[1] = 0.0f;
  tgtRate[2] = 0.0f;
  pidLoopHz = 0;
  mag_comp_x = 0.0f;
  mag_comp_y = 0.0f;
  mag_comp_z = 0.0f;
  magTelemX = 0.0f;
  magTelemY = 0.0f;
  magTelemZ = 0.0f;
  iTermRoll = 0.0f;
  iTermPitch = 0.0f;
  iTermYaw = 0.0f;

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
}

void injectImu(float roll_deg, float pitch_deg, float body_gz_dps,
               uint32_t tick) {
  const float roll = roll_deg * kDegToRad;
  const float pitch = pitch_deg * kDegToRad;
  const float body_ax = -std::sin(pitch);
  const float body_ay = std::sin(roll) * std::cos(pitch);
  const float body_az = std::cos(roll) * std::cos(pitch);

  inv_imu_sensor_event_t event1 = {};
  event1.gyro[2] = rawValue(-body_gz_dps / GYRO_SCALE);
  event1.accel[0] = rawValue(-body_ay / ACCEL_SCALE);
  event1.accel[1] = rawValue(body_ax / ACCEL_SCALE);
  event1.accel[2] = rawValue(body_az / ACCEL_SCALE);

  // Keep the freeze monitor live without adding a mean gyro bias.
  const int dither = (tick & 1U) ? -1 : 1;
  event1.gyro[0] = static_cast<int16_t>(dither);
  event1.accel[2] = static_cast<int16_t>(event1.accel[2] + 2 * dither);

  inv_imu_sensor_event_t event2 = {};
  for (int axis = 0; axis < 3; axis++) {
    const int sign = IMU2_SIGN[axis] < 0.0f ? -1 : 1;
    event2.gyro[axis] = static_cast<int16_t>(sign * event1.gyro[axis]);
    event2.accel[axis] = static_cast<int16_t>(sign * event1.accel[axis]);
  }
  IMU1.next_event = event1;
  IMU2.next_event = event2;
  IMU1.read_status = 0;
  IMU2.read_status = 0;
}

#ifdef SIL_YAW_OFF_TRACE_ONLY

int runOffTrace() {
  resetCommonFirmwareState(0.0f, 0.0f, 10.0f);
  yaw_enabled = true;
  targetAngleZ = -5.0f;

  constexpr uint32_t kTicks = 2000;
  arduino_fake::pre_tick_hook = [](uint32_t tick) {
    if (tick % 250U == 0U) {
      std::cout << tick << ',' << std::setprecision(9) << angleZ << ','
                << tgtRate[2] << ',' << motorOut[0] << ',' << motorOut[1]
                << ',' << motorOut[2] << ',' << motorOut[3] << '\n';
    }
    arduino_fake::millis_value += 1U;
    arduino_fake::micros_value += 1000U;
    lastRcMs = millis();
    injectImu(0.0f, 0.0f, 0.8f, tick);
  };
  arduino_fake::tick_limit = kTicks;
  try {
    pid_task(nullptr);
  } catch (const arduino_fake::TaskDelayExit &) {
  }
  arduino_fake::pre_tick_hook = nullptr;
  arduino_fake::tick_limit = 0;
  std::cout << kTicks << ',' << std::setprecision(9) << angleZ << ','
            << tgtRate[2] << ',' << motorOut[0] << ',' << motorOut[1] << ','
            << motorOut[2] << ',' << motorOut[3] << '\n';
  return 0;
}

#else

struct BodyMag {
  float x;
  float y;
  float z;
};

BodyMag syntheticBodyMag(float roll_deg, float pitch_deg,
                         float heading_deg) {
  const float roll = roll_deg * kDegToRad;
  const float pitch = pitch_deg * kDegToRad;
  const float heading = heading_deg * kDegToRad;
  const float sr = std::sin(roll);
  const float cr = std::cos(roll);
  const float sp = std::sin(pitch);
  const float cp = std::cos(pitch);
  const float sh = std::sin(heading);
  const float ch = std::cos(heading);

  // Inverse of Rz(heading) * Ry(pitch) * Rx(roll), for a horizontal
  // navigation-frame magnetic vector [1, 0, 0].
  return {
      cp * ch,
      sr * sp * ch - cr * sh,
      cr * sp * ch + sr * sh,
  };
}

struct YawRun {
  float fused_yaw_deg;
  float true_yaw_frame_deg;
  float target_rate_yaw_dps;
  int motor_yaw_differential;
};

YawRun runYawFusion(float roll_deg, float pitch_deg,
                    float true_heading_start_deg, float estimate_start_deg,
                    float true_rate_dps, float gyro_bias_dps,
                    uint32_t ticks, bool enable_mag,
                    bool enable_yaw_hold = false,
                    uint32_t error_injection_tick =
                        std::numeric_limits<uint32_t>::max(),
                    float error_injection_deg = 0.0f) {
  resetCommonFirmwareState(roll_deg, pitch_deg, estimate_start_deg);
  mag_enabled = enable_mag;
  mag_ready = true;
  magHeading = 0.0f;
  magSampleValid = false;
  magSampleMs = 0;
  mag_reference_pending = true;
  yaw_enabled = enable_yaw_hold;
  targetAngleZ = estimate_start_deg;

  arduino_fake::pre_tick_hook =
      [=](uint32_t tick) {
        arduino_fake::millis_value += 1U;
        arduino_fake::micros_value += 1000U;
        lastRcMs = millis();

        const float true_heading =
            true_heading_start_deg + true_rate_dps * (tick * kDt);
        injectImu(roll_deg, pitch_deg, true_rate_dps + gyro_bias_dps, tick);
        if (tick % 20U == 0U) {
          const BodyMag mag =
              syntheticBodyMag(roll_deg, pitch_deg, true_heading);
          publishMagSample(mag.x, mag.y, mag.z, millis());
        }
        if (tick == error_injection_tick) {
          angleZ += error_injection_deg;
        }
      };
  arduino_fake::tick_limit = ticks;

  bool stopped = false;
  try {
    pid_task(nullptr);
  } catch (const arduino_fake::TaskDelayExit &) {
    stopped = true;
  }
  arduino_fake::pre_tick_hook = nullptr;
  arduino_fake::tick_limit = 0;
  CHECK_MSG(stopped, "pid_task did not stop at the requested tick limit");

  return {
      angleZ,
      estimate_start_deg + true_rate_dps * (ticks * kDt),
      tgtRate[2],
      (motorOut[2] + motorOut[3]) - (motorOut[0] + motorOut[1]),
  };
}

std::string errorDetail(const char *label, float fused_error,
                        float reference_error = 0.0f) {
  std::ostringstream detail;
  detail << label << " fused_error=" << fused_error;
  if (reference_error != 0.0f) {
    detail << " gyro_only_error=" << reference_error;
  }
  return detail.str();
}

int runFusionTests() {
  std::cout << std::fixed << std::setprecision(4);

  runCase("convergence on a rotating heading", [] {
    const YawRun result =
        runYawFusion(0.0f, 0.0f, 40.0f, 0.0f,
                     12.0f, 0.0f, 24000U, true,
                     false, 1000U, 30.0f);
    const float error = std::fabs(
        wrapDeg(result.true_yaw_frame_deg - result.fused_yaw_deg));
    std::cout << "[MAG-SIL] convergence true_relative="
              << result.true_yaw_frame_deg
              << " fused=" << result.fused_yaw_deg
              << " error=" << error << "deg\n";
    CHECK_MSG(error < 1.0f, errorDetail("rotating convergence", error));

    const YawRun hold =
        runYawFusion(0.0f, 0.0f, 90.0f, 0.0f,
                     0.0f, 0.0f, 10000U, true, true);
    const float hold_error =
        std::fabs(wrapDeg(hold.true_yaw_frame_deg - hold.fused_yaw_deg));
    std::cout << "[MAG-SIL] relative-hold physical_heading=90.0000"
              << " yaw_error=" << hold_error
              << "deg target_rate=" << hold.target_rate_yaw_dps
              << "dps motor_yaw_diff=" << hold.motor_yaw_differential
              << "us\n";
    CHECK_MSG(hold_error < 0.1f,
              errorDetail("relative yaw-hold angle", hold_error));
    CHECK_MSG(std::fabs(hold.target_rate_yaw_dps) < 0.1f,
              "relative yaw hold generated a false target rate");
    CHECK_MSG(std::abs(hold.motor_yaw_differential) <= 2,
              "relative yaw hold generated a false motor yaw differential");
  });

  runCase("gyro bias drift rejection", [] {
    const YawRun fused =
        runYawFusion(0.0f, 0.0f, 25.0f, 25.0f,
                     0.0f, 0.6f, 30000U, true);
    const YawRun gyro_only =
        runYawFusion(0.0f, 0.0f, 25.0f, 25.0f,
                     0.0f, 0.6f, 30000U, false);
    const float fused_error =
        std::fabs(wrapDeg(fused.true_yaw_frame_deg - fused.fused_yaw_deg));
    const float gyro_error =
        std::fabs(wrapDeg(gyro_only.true_yaw_frame_deg -
                          gyro_only.fused_yaw_deg));
    std::cout << "[MAG-SIL] drift fused_error=" << fused_error
              << "deg gyro_only_error=" << gyro_error << "deg\n";
    CHECK_MSG(fused_error < 3.0f,
              errorDetail("bias rejection", fused_error, gyro_error));
    CHECK_MSG(gyro_error > 12.0f,
              errorDetail("gyro-only negative control", fused_error,
                          gyro_error));
    CHECK_MSG(fused_error * 3.0f < gyro_error,
              errorDetail("fusion improvement", fused_error, gyro_error));
  });

  runCase("tilt compensation at nonzero roll and pitch", [] {
    constexpr float kRoll = 27.0f;
    constexpr float kPitch = -19.0f;
    constexpr float kTrueHeading = 73.0f;
    const BodyMag mag = syntheticBodyMag(kRoll, kPitch, kTrueHeading);
    const float measured_heading = tiltCompensatedMagHeadingDeg(
        mag.x, mag.y, mag.z, kRoll, kPitch);
    const float heading_error =
        std::fabs(wrapDeg(kTrueHeading - measured_heading));

    const YawRun result =
        runYawFusion(kRoll, kPitch, kTrueHeading, kTrueHeading,
                     0.0f, 0.0f, 24000U, true,
                     false, 1000U, 30.0f);
    const float fused_error = std::fabs(
        wrapDeg(result.true_yaw_frame_deg - result.fused_yaw_deg));
    std::cout << "[MAG-SIL] tilt heading=" << measured_heading
              << " direct_error=" << heading_error
              << "deg fused_error=" << fused_error << "deg\n";
    CHECK_MSG(heading_error < 0.05f,
              errorDetail("tilt-comp direct", heading_error));
    CHECK_MSG(fused_error < 1.0f,
              errorDetail("tilted convergence", fused_error));

    resetCommonFirmwareState(kRoll, kPitch, kTrueHeading);
    mag_enabled = true;
    mag_ready = true;
    magSampleValid = false;
    mag_reference_pending = true;
    mag_comp_x = 0.0020f;
    mag_comp_y = -0.0010f;
    mag_comp_z = 0.0015f;
    const float throttle_delta =
        static_cast<float>(base_throttle - MAG_THROTTLE_REF_US);
    publishMagSample(
        mag.x + mag_comp_x * throttle_delta,
        mag.y + mag_comp_y * throttle_delta,
        mag.z + mag_comp_z * throttle_delta,
        millis());
    injectImu(kRoll, kPitch, 0.0f, 0U);
    arduino_fake::pre_tick_hook = [](uint32_t) {
      arduino_fake::millis_value += 1U;
      arduino_fake::micros_value += 1000U;
      lastRcMs = millis();
    };
    arduino_fake::tick_limit = 1U;
    try {
      pid_task(nullptr);
    } catch (const arduino_fake::TaskDelayExit &) {
    }
    arduino_fake::pre_tick_hook = nullptr;
    arduino_fake::tick_limit = 0U;

    CHECK_NEAR(magTelemX, mag.x, 1e-6f);
    CHECK_NEAR(magTelemY, mag.y, 1e-6f);
    CHECK_NEAR(magTelemZ, mag.z, 1e-6f);
    CHECK_NEAR(magHeading, measured_heading, 0.05f);
  });

  runCase("+/-180 degree wrap uses the short positive correction", [] {
    const float correction = magYawCorrectionDeg(179.0f, -179.0f, K_MAG);
    const float wrong_long_way =
        magYawCorrectionDeg(-179.0f, 179.0f, K_MAG);
    std::cout << "[MAG-SIL] wrap correction(+2deg)=" << correction
              << " opposite(-2deg)=" << wrong_long_way << '\n';
    CHECK(correction > 0.0f);
    CHECK(wrong_long_way < 0.0f);
    CHECK_NEAR(correction, 2.0f * K_MAG, 1e-7f);
    CHECK_NEAR(wrong_long_way, -2.0f * K_MAG, 1e-7f);
  });

  std::cout << "\n" << (test_count - failure_count) << "/" << test_count
            << " BMM350 yaw-fusion SIL cases passed\n";
  return failure_count == 0 ? 0 : 1;
}

#endif

}  // namespace

int main() {
  static_assert(sizeof(long) == 4,
                "BMM350 SIL requires ESP32-width 32-bit long");
#ifdef SIL_YAW_OFF_TRACE_ONLY
  return runOffTrace();
#else
  return runFusionTests();
#endif
}
