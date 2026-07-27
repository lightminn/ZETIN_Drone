#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <ICM42670P.h>
#include <DFRobot_BMM350.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_task_wdt.h>
#include <errno.h>
#include <float.h>

#include "mag_yaw_fusion.h"
#include "yaw_command.h"
#include "failsafe_land.h"

#ifndef WIFI_LATENCY_DEBUG
#define WIFI_LATENCY_DEBUG 0
#endif
#if WIFI_LATENCY_DEBUG
#ifndef WIFI_LATENCY_UDP_THRESHOLD_US
#define WIFI_LATENCY_UDP_THRESHOLD_US 30000U
#endif
#ifndef WIFI_LATENCY_PID_THRESHOLD_US
#define WIFI_LATENCY_PID_THRESHOLD_US 5000U
#endif

enum LatKind : uint8_t {
  LAT_UDP_LOOP,
  LAT_UDP_PARSE,
  LAT_UDP_SEND,
  LAT_PID
};

struct LatEvt {
  uint32_t t_ms;
  uint32_t dur_us;
  uint8_t kind;
};

static const uint32_t LAT_RING_SIZE = 128;
static const uint32_t LAT_RING_MASK = LAT_RING_SIZE - 1;

struct LatRing {
  LatEvt events[LAT_RING_SIZE];
  volatile uint32_t head = 0;
  volatile uint32_t tail = 0;
  volatile uint32_t dropped = 0;
};

static LatRing udpLatRing;
static LatRing pidLatRing;

static uint32_t latUdpMaxUs = 0;
static uint32_t latPidMaxUs = 0;
static uint32_t latUdpLoopOver30 = 0;
static uint32_t latUdpLoopOver100 = 0;
static uint32_t latUdpLoopOver300 = 0;
static uint32_t latPidOver5 = 0;
static uint32_t latLastSummaryMs = 0;
#endif
// ==========================================================
// 1. 튜닝 파라미터
// ==========================================================
// [Outer] 각도 P: 각도 오차 -> 목표 각속도(dps)
volatile float Kp_Angle_Roll  = 6.0f;
volatile float Kp_Angle_Pitch = 6.0f;
volatile float Kp_Angle_Yaw   = 3.0f;

// [Inner] 각속도 PID: 각속도 오차 -> 모터 파워
// Ki_Rate_Roll/Pitch: 0.005 -> 0.05 (호스트 SIL 근거, commit 735f77d). 0.005는
// 일정 CG 치우침 외란에서 10초 후 ±50µs 클램프의 ~1.4%만 써 적분기가 사실상
// 비활성이었다(파라미터 무관하게 강건한 결론). 0.05는 적분기를 활성화하되
// 클램프의 <5µs만 쓰는 보수적 시작값이다. SIL의 완전트림 값(0.5)은 실기 지연·
// 노이즈에서 진동 위험이 있어 일부러 피했고, 최종값은 테더 벤치에서 상향 튜닝한다.
volatile float Kp_Rate_Roll  = 0.50f, Ki_Rate_Roll  = 0.05f, Kd_Rate_Roll  = 0.015f;
volatile float Kp_Rate_Pitch = 0.50f, Ki_Rate_Pitch = 0.05f, Kd_Rate_Pitch = 0.015f;
volatile float Kp_Rate_Yaw   = 1.50f, Ki_Rate_Yaw   = 0.05f,  Kd_Rate_Yaw   = 0.0f;

volatile int  base_throttle = 1000;
volatile int  min_throttle  = 1050;
volatile int  max_throttle  = 1300;
volatile bool yaw_hold_override = false;   // "yaw 1": heading 고정 + 재슬레이빙 금지
volatile bool yaw_hold_now = false;   // 텔레메트리용: 현재 heading 잠금 중인가

const int MAG_THROTTLE_REF_US = 1000;   // 보정 기준(모터 off 근처)
// 모터 전류 간섭 보정: body-frame ΔB(µT)는 base_throttle에 비례. raw mag에서 뺀다.
// mag.x -= mag_comp_x*(base_throttle-REF) 등. µT/µs. 런타임 `magc x y z`. 0=off.
// 벤치 특성화 2026-07-27 (log 023805, 검증 024330): throttle 간섭 +3.64→+0.02°/100µs.
volatile float mag_comp_x =  0.007497f;  // µT/µs
volatile float mag_comp_y = -0.001218f;
volatile float mag_comp_z = -0.000640f;

// ==========================================================
// 2. 시스템 상수
// ==========================================================
const char* WIFI_SSID    = "Drone_Tuning";
const char* WIFI_PASS    = "12345678";
const int   UDP_PORT     = 4210;
const int   WIFI_CHANNEL = 11;  // ch6 혼잡(인근 AP 다수) → 스캔상 가장 빈 채널로 이동

const int pinM1   = 4;   // FL
const int pinM2   = 5;   // RR
const int pinM3   = 6;   // FR
const int pinM4   = 7;   // RL
const int SPI_CS1 = 10;  // IMU1
const int SPI_CS2 = 9;   // IMU2
const int BMM_SDA = 35;
const int BMM_SCL = 36;
const int LDO_SHD = 38;

const int ESC_FREQ   = 400;
const int ESC_RES    = 14;
const int ESC_PERIOD = 2500;                 // us
const int ESC_MAXDUTY = (1 << ESC_RES) - 1;  // magic number 제거

const float GYRO_SCALE   = 1.0f / 16.4f;     // raw -> deg/s
const float ACCEL_SCALE  = 1.0f / 2048.0f;   // raw -> g
const float SAFETY_ANGLE = 60.0f;
const float YAW_DEADZONE = 0.3f;
const float MAX_TARGET_ANGLE_RP = 30.0f;     // UDP 오입력에 대한 자세 명령 제한
const float TRIM_MAX_DEG = 10.0f;   // 트림 절대값 상한
const float MAX_TARGET_RATE_RP  = 300.0f;    // outer-loop 출력 제한 (deg/s)
const float MAX_TARGET_RATE_YAW = 180.0f;
// yaw 모드 판정 임계치. 벤치에서 조정한다(설계 문서 §1 참조).
const float YAW_RATE_DEADZONE   = 3.0f;    // 명령 각속도 절댓값이 이하면 스틱 중립
const float YAW_HOLD_SETTLE_DPS = 10.0f;   // bodyGz 절댓값이 이하면 정착
// 250Hz outer loop에서 K=0.001 -> tau ≈ 0.004/K = 4s.
const float K_MAG = 0.001f;
const uint32_t MAG_SAMPLE_PERIOD_MS = 20;    // BMM350 read = 50Hz, core 0 only
const uint32_t MAG_STALE_MS = 100;

// magcal 출력값을 아래 세 상수에 붙여 넣는다. 보정은 raw - offset.
// 2026-07-24 벤치 캘리브 (magcal, samples=3454).
const float MAG_HARD_IRON_OFFSET_X = -1.831376f;
const float MAG_HARD_IRON_OFFSET_Y = 9.318241f;
const float MAG_HARD_IRON_OFFSET_Z = -12.517762f;

// BMM350 축이 기체 축과 다르면 벤치 sign test 뒤 여기만 바꾼다.
const float MAG_BODY_SIGN_X = 1.0f;
const float MAG_BODY_SIGN_Y = 1.0f;
const float MAG_BODY_SIGN_Z = 1.0f;

// IMU2는 x,z축이 IMU1 대비 반전 (y는 동일). 첫 파일 기준 그대로.
const float IMU2_SIGN_X = -1.0f;
const float IMU2_SIGN_Y =  1.0f;
const float IMU2_SIGN_Z = -1.0f;
static const float IMU1_SIGN[3] = { 1.0f, 1.0f, 1.0f };
static const float IMU2_SIGN[3] = { IMU2_SIGN_X, IMU2_SIGN_Y, IMU2_SIGN_Z };

// --- 안전/redundancy 임계값 (하드웨어 맞춰 튜닝 필요) ---
const uint32_t RC_TIMEOUT_MS      = 500;
// 아래 failsafe 상수는 전부 벤치 조정 대상이다.
constexpr int CTRL_MARGIN = 150;
const int      FS_DESCENT_DELTA_US = 60;
static_assert(FS_DESCENT_DELTA_US < CTRL_MARGIN,
              "FS_DESCENT_DELTA_US must be less than CTRL_MARGIN");
// 컴파일 타임 가드는 delta < CTRL_MARGIN만 검사한다. 실제 하한은
// delta < min(CTRL_MARGIN, entry_throttle - 1050)이며 entry_throttle은
// 런타임 값이라 static_assert로 잡을 수 없다. 예: entry=1150이면 delta=120도
// 이미 collective 하한에 걸려 실제 하강이 일어나지 않는다.
const uint32_t FS_MIN_DESCEND_MS   = 1000;
const float    FS_LAND_LPF_ALPHA   = 0.03f;   // 1kHz에서 약 5Hz
// 능동 프로브 상수는 모두 Stage E 벤치 조정 대상이다. 호버 1340us는 아이들
// 1000us 대비 340us이고, 40us 딥은 추력 11.8%=0.118g다. 120ms 동안 추가
// 하강 거리는 약 0.008m다. 실측 LPF 잔차 노이즈는 0.047g지만 지배 성분이
// 5Hz 아래라 400ms 안의 딥 직전 대비 차분에서는 거의 공통모드로 상쇄된다.
const int      FS_PROBE_DIP_US          = 40;
const uint32_t FS_PROBE_PERIOD_MS       = 400;
const uint32_t FS_PROBE_DIP_MS          = 120;
const uint32_t FS_PROBE_SAMPLE_DELAY_MS = 30;
const float    FS_PROBE_RESPONSE_G      = 0.06f;
const uint8_t  FS_PROBE_CONFIRM_N       = 2;
const uint32_t FS_MAX_MS           = 5000;
static_assert(FS_PROBE_DIP_US < CTRL_MARGIN,
              "FS_PROBE_DIP_US must be less than CTRL_MARGIN");
static_assert(FS_PROBE_SAMPLE_DELAY_MS < FS_PROBE_DIP_MS,
              "probe sample delay must be shorter than the dip");
static_assert(FS_PROBE_DIP_MS < FS_PROBE_PERIOD_MS,
              "probe dip must finish before the next probe");
const LandProbeConfig FS_LAND_PROBE_CONFIG = {
    FS_LAND_LPF_ALPHA,
    FS_MIN_DESCEND_MS,
    FS_PROBE_PERIOD_MS,
    FS_PROBE_DIP_MS,
    FS_PROBE_SAMPLE_DELAY_MS,
    FS_PROBE_RESPONSE_G,
    FS_PROBE_CONFIRM_N,
    FS_PROBE_DIP_US,
};
// 아래 호버 추정 상수는 모두 Stage E-0 벤치 조정 대상이다. 10°/±0.05g는
// 자세·수직 정상상태만 받는 보수적 시작값이고, 3초 LPF는 요구 범위(2~5초)의
// 중앙값이다. 1.5초 누적은 짧은 스로틀 통과를 호버로 확정하지 않으면서
// 실측 상한 모델인 0.06g 저주파 진동에서도 3초 안에 유효해지는 시작값이다.
const float    HOVER_LEVEL_MAX_DEG    = 10.0f;
const float    HOVER_ACCEL_TOL_G      = 0.05f;
const int      HOVER_MIN_THROTTLE_US  = 1150;
const float    HOVER_LPF_TAU_S        = 3.0f;
const uint32_t HOVER_VALID_MS         = 1500;
// 기존 1200us 컷은 가정 호버 1340us에서 margin 140us였다. 두 번의 60us
// 하강 delta에 해당하는 120us로 보수적으로 시작하고 실측 hover_est로 조정한다.
const int      FS_GROUND_MARGIN_US    = 120;
const uint32_t PID_WDT_TIMEOUT_MS = 500;     // pid_task 정지 시 강제 재부팅 (1kHz 루프 대비 큰 여유)
const int32_t  FROZEN_DELTA_RAW   = 1;       // 6축 raw 변화량 합이 이 이하면 정지 의심 (LSB)
const uint32_t IMU_FROZEN_MS      = 300;     // 그 상태가 이만큼 지속되면 freeze 확정
const float    GYRO_DISAGREE_DPS  = 15.0f;   // 두 IMU 각속도 차이 임계 (deg/s)
const uint32_t IMU_DISAGREE_MS    = 150;     // 불일치 지속 시간

const int   BIAS_CALIB_SAMPLES   = 2000;
const float BIAS_MOVEMENT_THRESH = 1.0f;     // 축별 stddev 상한 (deg/s)
const int   BIAS_CALIB_RETRIES   = 3;
const float ACCEL_CAL_MIN_G      = 0.8f;     // 이 범위 밖은 센서 이상: 보정 금지
const float ACCEL_CAL_MAX_G      = 1.2f;

const int OUTER_DIV = 4;                      // outer loop = 1kHz / 4 = 250Hz

// --- 필터/적분 상한 ---
const float GYRO_SW_LPF_HZ = 80.0f;  // 하드웨어 LPF(121Hz)에 겹치는 소프트웨어 LPF
const float I_TERM_MAX_US  = 50.0f;  // 축별 적분 기여 상한 (모터 µs)

// ==========================================================
// 3. 유틸
// ==========================================================
class LowPassFilter {
public:
  float alpha, last = 0.0f;
  LowPassFilter(float cutoff_hz, float dt) {
    float rc = 1.0f / (2.0f * PI * cutoff_hz);
    alpha = dt / (rc + dt);
  }
  float update(float in) { last += alpha * (in - last); return last; }
  void reset(float value = 0.0f) { last = value; }
};

// Arduino sketch preprocessor의 자동 함수 원형보다 먼저 보여야 하는 반환 타입.
struct MotorMix {
  int motor[4];
  bool scaled;
};

// IMU별 freeze 감시 (raw 레지스터 값이 멈췄는지)
struct FreezeMon {
  int16_t  lastGyro[3] = {0, 0, 0};
  int16_t  lastAccel[3] = {0, 0, 0};
  uint32_t since = 0;
  bool     init  = false;
};
#if WIFI_LATENCY_DEBUG
static inline void recordLatEvent(LatRing &ring, uint32_t tMs,
                                  uint32_t durUs, LatKind kind) {
  uint32_t head = __atomic_load_n(&ring.head, __ATOMIC_RELAXED);
  uint32_t tail = __atomic_load_n(&ring.tail, __ATOMIC_ACQUIRE);
  if (head - tail >= LAT_RING_SIZE) {
    __atomic_fetch_add(&ring.dropped, 1, __ATOMIC_RELAXED);
    return;
  }
  ring.events[head & LAT_RING_MASK] = {tMs, durUs, (uint8_t)kind};
  __atomic_store_n(&ring.head, head + 1, __ATOMIC_RELEASE);
}

static inline bool popLatEvent(LatRing &ring, LatEvt &event) {
  uint32_t tail = __atomic_load_n(&ring.tail, __ATOMIC_RELAXED);
  if (tail == __atomic_load_n(&ring.head, __ATOMIC_ACQUIRE)) return false;
  event = ring.events[tail & LAT_RING_MASK];
  __atomic_store_n(&ring.tail, tail + 1, __ATOMIC_RELEASE);
  return true;
}

static const char *latKindName(uint8_t kind) {
  switch (kind) {
    case LAT_UDP_LOOP:  return "UDP_LOOP";
    case LAT_UDP_PARSE: return "UDP_PARSE";
    case LAT_UDP_SEND:  return "UDP_SEND";
    case LAT_PID:       return "PID";
    default:            return "UNKNOWN";
  }
}

static void drainLatRing(LatRing &ring) {
  LatEvt event;
  while (popLatEvent(ring, event)) {
    if (event.kind == LAT_PID) {
      if (event.dur_us > latPidMaxUs) latPidMaxUs = event.dur_us;
      if (event.dur_us > 5000U) latPidOver5++;
    } else {
      if (event.dur_us > latUdpMaxUs) latUdpMaxUs = event.dur_us;
      if (event.kind == LAT_UDP_LOOP) {
        if (event.dur_us > 30000U) latUdpLoopOver30++;
        if (event.dur_us > 100000U) latUdpLoopOver100++;
        if (event.dur_us > 300000U) latUdpLoopOver300++;
      }
    }
    Serial.printf("[LAT] t=%lu kind=%s dur_us=%lu\n",
                  (unsigned long)event.t_ms, latKindName(event.kind),
                  (unsigned long)event.dur_us);
  }
}
#endif
static bool checkFreeze(FreezeMon &m, const inv_imu_sensor_event_t &e, uint32_t nowMs) {
  if (!m.init) {
    for (int k = 0; k < 3; k++) {
      m.lastGyro[k] = e.gyro[k];
      m.lastAccel[k] = e.accel[k];
    }
    m.init = true;
    return false;
  }

  // abs(x)+abs(y)+abs(z)의 변화만 보면 축 변화가 서로 상쇄될 수 있다.
  // 각 레지스터의 변화량을 직접 더해 통신 정지/동일 프레임 반복을 감시한다.
  int32_t delta = 0;
  for (int k = 0; k < 3; k++) {
    delta += abs((int32_t)e.gyro[k] - m.lastGyro[k]);
    delta += abs((int32_t)e.accel[k] - m.lastAccel[k]);
  }

  bool frozen = false;
  if (delta <= FROZEN_DELTA_RAW) {
    if (m.since == 0) m.since = nowMs;
    else if (nowMs - m.since >= IMU_FROZEN_MS) frozen = true;
  } else {
    m.since = 0;
  }
  for (int k = 0; k < 3; k++) {
    m.lastGyro[k] = e.gyro[k];
    m.lastAccel[k] = e.accel[k];
  }
  return frozen;
}

// RC 워치독. nowMs는 루프 맨 위에서 뜬 값이라 udp_task(코어0)가 그 뒤에
// lastRcMs를 갱신하면 lastRcMs가 nowMs보다 미래일 수 있다. 부호 있는 차이로
// 비교해야 unsigned 언더플로로 즉시 타임아웃이 터지지 않는다. millis() wrap도
// 모듈러 뺄셈이라 그대로 안전하다.
static inline bool rcTimedOut(uint32_t nowMs, uint32_t lastMs) {
  return (int32_t)(nowMs - lastMs) > (int32_t)RC_TIMEOUT_MS;
}

// ==========================================================
// 4. 시스템 변수
// ==========================================================
WiFiUDP   udp;
char      packetBuffer[256];
IPAddress laptopIP;
int       laptopPort            = 0;
bool      connectionEstablished = false;

// startGyro/startAccel은 ODR·FSR만 설정하고 UI 필터 대역폭은 칩 전원
// 기본값(bypass) 그대로 둔다. 그러면 프롭 진동이 필터 없이 PID에 들어오므로
// protected icm_driver에 접근해 하드웨어 LPF를 켠다 (gyro 121Hz, accel 25Hz).
class ICM42670WithLPF : public ICM42670 {
public:
  ICM42670WithLPF(SPIClass &spi_bus, uint8_t cs) : ICM42670(spi_bus, cs) {}
  int setLowPassFilters() {
    int rc = 0;
    rc |= inv_imu_set_gyro_ln_bw(&icm_driver, GYRO_CONFIG1_GYRO_FILT_BW_121);
    rc |= inv_imu_set_accel_ln_bw(&icm_driver, ACCEL_CONFIG1_ACCEL_FILT_BW_25);
    return rc;
  }
};

ICM42670WithLPF IMU1(SPI, SPI_CS1);
ICM42670WithLPF IMU2(SPI, SPI_CS2);
DFRobot_BMM350_I2C bmm(&Wire, 0x14);

volatile bool  safety_lock  = true;
// 런타임 safety_lock 전이는 Core 1(pid_task)만 수행한다. Core 0은 요청만
// 게시하며, 두 요청이 함께 대기하면 disarm이 arm보다 우선한다.
volatile bool  safety_disarm_requested = false;
volatile bool  safety_arm_requested = false;
volatile bool  failsafe_resume_requested = false;
portMUX_TYPE safetyRequestMux = portMUX_INITIALIZER_UNLOCKED;
// 런타임 쓰기는 pid_task만 담당한다. Core 0은 텔레메트리/명령 처리에서 읽기만 한다.
volatile uint8_t fs_phase = FS_NONE;   // 텔레메트리 Failsafe_Phase
volatile uint8_t fs_probe_state = FS_PROBE_WAIT;
volatile uint8_t fs_probe_no_response = 0;
volatile float fs_probe_response_g = 0.0f;
volatile float hover_est = 0.0f;       // 추정 호버 collective (us)
volatile bool hover_valid = false;
// 비행 중에는 pid_task만 쓰고, start는 safety_lock=true인 동안 초기화한 뒤
// 마지막에 Core 1에 arm 전이를 요청한다.
HoverThrottleEstimator hoverTracker = {};
volatile float targetAngleX = 0.0f, targetAngleY = 0.0f, targetAngleZ = 0.0f;
// 기체 트림(도). 추정기 0°와 진짜 수평의 차이를 보정한다. 비행별 상태가 아니라
// 기체 속성이므로 start/stop이 지우지 않는다.
volatile float trim_roll = 0.0f;
volatile float trim_pitch = 0.0f;
volatile float targetYawRate = 0.0f;   // rcr이 준 yaw 각속도 명령 (dps)

volatile float angleX = 0.0f, angleY = 0.0f, angleZ = 0.0f; // 추정 각도
volatile float gyroX  = 0.0f, gyroY  = 0.0f, gyroZ  = 0.0f; // 융합 각속도 (body frame)
volatile float accX   = 0.0f, accY   = 0.0f, accZ   = 0.0f; // 융합 가속도 (g)
volatile int   motorOut[4] = {1000,1000,1000,1000};
volatile float tgtRate[3]  = {0,0,0}; // roll,pitch,yaw dps
volatile int   pidLoopHz   = 0;

// Core 0 writer -> Core 1 reader. 짝수 seq만 완성된 벡터다.
volatile bool  mag_enabled = false;
volatile bool  mag_ready = false;
volatile bool  mag_calibrating = false;
volatile float magFieldX = 0.0f, magFieldY = 0.0f, magFieldZ = 0.0f;
volatile float magHeading = 0.0f;
volatile float magTelemX = 0.0f, magTelemY = 0.0f, magTelemZ = 0.0f;
volatile uint32_t magSampleMs = 0;
volatile bool magSampleValid = false;
volatile bool mag_reference_pending = true;

portMUX_TYPE magSnapshotMux = portMUX_INITIALIZER_UNLOCKED;
float magYawReferenceOffset = 0.0f;

float magCalMin[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
float magCalMax[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
uint32_t magCalSampleCount = 0;
uint32_t lastMagReadMs = 0;

float iTermRoll = 0.0f, iTermPitch = 0.0f, iTermYaw = 0.0f;  // 적분 기여 (모터 µs)

float gyro_bias1[3] = {0,0,0};
float gyro_bias2[3] = {0,0,0};
// 두 센서의 gain 오차는 독립이고 단일-IMU 폴백도 있으므로 각각 보정한다.
float accel_scale1 = 1.0f;
float accel_scale2 = 1.0f;

volatile uint32_t lastRcMs        = 0;
volatile bool     fault_rc        = false;
volatile bool     fault_imu1      = false;   // IMU1 freeze
volatile bool     fault_imu2      = false;   // IMU2 freeze
volatile bool     fault_disagree  = false;   // 두 IMU 불일치 (중재 불가)
volatile bool     fault_attitude  = false;   // 과도 기울기
volatile int      active_imus     = 2;       // 현재 사용 중인 IMU 수 (telemetry)
volatile bool     mixer_scaled    = false;   // 자세 mixer가 축소됐는지
volatile bool     calibration_ok  = false;

// 재시동 판단용 현재 센서 상태. fault_imu* / fault_disagree는 비행 중 latch된다.
volatile bool     imu1_frozen_now = false;
volatile bool     imu2_frozen_now = false;
volatile bool     imu_disagree_now = false;

// scripts/control_dualsense.py 프로토콜 호환용
volatile uint32_t lastRcSeq       = 0;
volatile bool     rcSeqValid      = false;
volatile uint32_t rcTotalPkts     = 0;
volatile uint32_t rcDroppedPkts   = 0;

static inline void requestSafetyDisarm() {
  portENTER_CRITICAL(&safetyRequestMux);
  safety_disarm_requested = true;
  safety_arm_requested = false;
  failsafe_resume_requested = false;
  portEXIT_CRITICAL(&safetyRequestMux);
}

static inline void requestSafetyArm() {
  portENTER_CRITICAL(&safetyRequestMux);
  if (!safety_disarm_requested) safety_arm_requested = true;
  portEXIT_CRITICAL(&safetyRequestMux);
}

static inline void applyPendingSafetyRequest() {
  portENTER_CRITICAL(&safetyRequestMux);
  if (safety_disarm_requested) {
    safety_lock = true;
    safety_disarm_requested = false;
    safety_arm_requested = false;
  } else if (safety_arm_requested) {
    safety_lock = false;
    safety_arm_requested = false;
  }
  portEXIT_CRITICAL(&safetyRequestMux);
}

static inline void requestFailsafeResume() {
  portENTER_CRITICAL(&safetyRequestMux);
  failsafe_resume_requested = true;
  portEXIT_CRITICAL(&safetyRequestMux);
}

static inline bool takeFailsafeResumeRequest() {
  portENTER_CRITICAL(&safetyRequestMux);
  const bool requested = failsafe_resume_requested;
  failsafe_resume_requested = false;
  portEXIT_CRITICAL(&safetyRequestMux);
  return requested;
}

static inline ResumeRefusalReason resumeRefusalReason(uint32_t nowMs) {
  if (fs_phase != FS_DESCENDING) return RESUME_REFUSED_PHASE;
  if (rcTimedOut(nowMs, lastRcMs)) return RESUME_REFUSED_RC;
  if (fabsf(angleX) > SAFETY_ANGLE || fabsf(angleY) > SAFETY_ANGLE) {
    return RESUME_REFUSED_TILT;
  }
  if (active_imus <= 0
      || (imu1_frozen_now && imu2_frozen_now)
      || imu_disagree_now) {
    return RESUME_REFUSED_IMU;
  }
  if (!hover_valid) return RESUME_REFUSED_HOVER;
  return RESUME_ALLOWED;
}

static inline void logResumeRefusal(ResumeRefusalReason reason) {
  Serial.printf(">>> RESUME REFUSED %s\n", resumeRefusalName(reason));
}

static inline void publishMagSample(
    float x, float y, float z, uint32_t sample_ms) {
  // I2C read happens before this lock. The critical section only publishes
  // four words, so pid_task never waits on the blocking transaction.
  portENTER_CRITICAL(&magSnapshotMux);
  magFieldX = x;
  magFieldY = y;
  magFieldZ = z;
  magSampleMs = sample_ms;
  magSampleValid = true;
  portEXIT_CRITICAL(&magSnapshotMux);
}

static inline bool readMagSnapshot(MagSnapshot &snapshot) {
  portENTER_CRITICAL(&magSnapshotMux);
  bool valid = magSampleValid;
  snapshot.x = magFieldX;
  snapshot.y = magFieldY;
  snapshot.z = magFieldZ;
  snapshot.sample_ms = magSampleMs;
  portEXIT_CRITICAL(&magSnapshotMux);
  return valid;
}

static bool initMagnetometer() {
  if (mag_ready) return true;

  pinMode(LDO_SHD, OUTPUT);
  digitalWrite(LDO_SHD, HIGH);
  delay(100);
  Wire.begin(BMM_SDA, BMM_SCL);
  uint8_t status = bmm.begin();
  if (status != 0) {
    Serial.printf("[MAG] BMM350 init failed (%u)\n", (unsigned)status);
    return false;
  }
  bmm.setOperationMode(eBmm350NormalMode);
  bmm.setRate(BMM350_DATA_RATE_50HZ);
  mag_ready = true;
  lastMagReadMs = millis() - MAG_SAMPLE_PERIOD_MS;
  Serial.println("[MAG] BMM350 ready @50Hz");
  return true;
}

static void startMagCalibration() {
  mag_enabled = false;
  mag_reference_pending = true;
  mag_calibrating = true;
  magCalMin[0] = FLT_MAX;
  magCalMin[1] = FLT_MAX;
  magCalMin[2] = FLT_MAX;
  magCalMax[0] = -FLT_MAX;
  magCalMax[1] = -FLT_MAX;
  magCalMax[2] = -FLT_MAX;
  magCalSampleCount = 0;
  Serial.println("[MAGCAL] rotate drone; send 'magcal 0' when finished");
}

static void stopMagCalibration() {
  mag_calibrating = false;
  if (magCalSampleCount == 0U) {
    Serial.println("[MAGCAL] no samples");
    return;
  }
  const float offset_x = 0.5f * (magCalMax[0] + magCalMin[0]);
  const float offset_y = 0.5f * (magCalMax[1] + magCalMin[1]);
  const float offset_z = 0.5f * (magCalMax[2] + magCalMin[2]);
  Serial.printf("[MAGCAL] samples=%lu\n", (unsigned long)magCalSampleCount);
  Serial.printf("const float MAG_HARD_IRON_OFFSET_X = %.6ff;\n", offset_x);
  Serial.printf("const float MAG_HARD_IRON_OFFSET_Y = %.6ff;\n", offset_y);
  Serial.printf("const float MAG_HARD_IRON_OFFSET_Z = %.6ff;\n", offset_z);
}

static void sampleMagnetometer(uint32_t now_ms) {
  if (!mag_ready || now_ms - lastMagReadMs < MAG_SAMPLE_PERIOD_MS) return;
  lastMagReadMs = now_ms;

  // Blocking I2C read: caller is udp_task on core 0, never pid_task.
  sBmm350MagData_t data = bmm.getGeomagneticData();
  const float raw[3] = {data.float_x, data.float_y, data.float_z};
  if (!isfinite(raw[0]) || !isfinite(raw[1]) || !isfinite(raw[2])) return;

  if (mag_calibrating) {
    for (int axis = 0; axis < 3; axis++) {
      if (raw[axis] < magCalMin[axis]) magCalMin[axis] = raw[axis];
      if (raw[axis] > magCalMax[axis]) magCalMax[axis] = raw[axis];
    }
    magCalSampleCount++;
  }

  publishMagSample(
      MAG_BODY_SIGN_X * (raw[0] - MAG_HARD_IRON_OFFSET_X),
      MAG_BODY_SIGN_Y * (raw[1] - MAG_HARD_IRON_OFFSET_Y),
      MAG_BODY_SIGN_Z * (raw[2] - MAG_HARD_IRON_OFFSET_Z),
      now_ms);
}

// ==========================================================
// 5. 모터
// ==========================================================
void writeMotor(int pin, int us) {
  us = constrain(us, 1000, 2000);
  uint32_t duty = ((uint32_t)us * ESC_MAXDUTY) / ESC_PERIOD;
  ledcWrite(pin, duty);
}
void stopMotors() {
  writeMotor(pinM1, 1000); writeMotor(pinM2, 1000);
  writeMotor(pinM3, 1000); writeMotor(pinM4, 1000);
}

// 자세 차동 명령을 먼저 보존하고 collective를 이동한다. 그래도 범위를 넘을 때만
// 모든 자세 명령을 같은 비율로 축소해 토크 비율을 유지한다.
static MotorMix mixAndDesaturate(float roll, float pitch, float yaw,
                                 int throttle, int minMotor, int maxMotor) {
  MotorMix out;
  minMotor = constrain(minMotor, 1000, 2000);
  maxMotor = constrain(maxMotor, minMotor, 2000);

  float diff[4] = {
    -pitch + roll - yaw,  // M1: FL
     pitch - roll - yaw,  // M2: RR
    -pitch - roll + yaw,  // M3: FR
     pitch + roll + yaw   // M4: RL
  };

  float minDiff = diff[0], maxDiff = diff[0];
  for (int i = 1; i < 4; i++) {
    minDiff = min(minDiff, diff[i]);
    maxDiff = max(maxDiff, diff[i]);
  }

  const float available = (float)(maxMotor - minMotor);
  const float span = maxDiff - minDiff;
  float scale = 1.0f;
  if (span > available && span > 0.0f) scale = available / span;
  out.scaled = scale < 0.9999f;

  if (out.scaled) {
    for (int i = 0; i < 4; i++) diff[i] *= scale;
    minDiff *= scale;
    maxDiff *= scale;
  }

  const float collectiveLo = minMotor - minDiff;
  const float collectiveHi = maxMotor - maxDiff;
  const float collective = min(max((float)throttle, collectiveLo), collectiveHi);

  for (int i = 0; i < 4; i++) {
    out.motor[i] = constrain((int)lroundf(collective + diff[i]), minMotor, maxMotor);
  }
  return out;
}

// ==========================================================
// 6. Gyro bias 캘리브레이션
// ==========================================================
// sign[]은 각 IMU를 drone(IMU1) frame으로 맞추는 부호.
static bool measure_imu_bias(ICM42670 &imu, const float sign[3],
                             float bias_out[3], float sd_out[3],
                             float &accel_mean_out) {
  double sum[3] = {0,0,0}, sum_sq[3] = {0,0,0};
  double accel_mag_sum = 0.0;
  inv_imu_sensor_event_t e = {};
  int samples = 0;
  int attempts = 0;
  const int maxAttempts = BIAS_CALIB_SAMPLES + 100;
  while (samples < BIAS_CALIB_SAMPLES && attempts < maxAttempts) {
    attempts++;
    if (imu.getDataFromRegisters(e) != 0) {
      delayMicroseconds(1000);
      continue;
    }
    float g[3] = {
      sign[0] * e.gyro[0] * GYRO_SCALE,
      sign[1] * e.gyro[1] * GYRO_SCALE,
      sign[2] * e.gyro[2] * GYRO_SCALE
    };
    for (int k = 0; k < 3; k++) { sum[k] += g[k]; sum_sq[k] += (double)g[k]*g[k]; }
    const double ax = (double)e.accel[0] * ACCEL_SCALE;
    const double ay = (double)e.accel[1] * ACCEL_SCALE;
    const double az = (double)e.accel[2] * ACCEL_SCALE;
    accel_mag_sum += sqrt(ax*ax + ay*ay + az*az);
    samples++;
    delayMicroseconds(1000);
  }
  if (samples != BIAS_CALIB_SAMPLES) return false;
  accel_mean_out = (float)(accel_mag_sum / samples);
  for (int k = 0; k < 3; k++) {
    double mean = sum[k] / samples;
    double var  = sum_sq[k] / samples - mean*mean;
    if (var < 0) var = 0;
    bias_out[k] = (float)mean;
    sd_out[k]   = (float)sqrt(var);
  }
  return true;
}

static float guarded_accel_scale(float accel_mean_g, const char *imu_name) {
  if (accel_mean_g < ACCEL_CAL_MIN_G || accel_mean_g > ACCEL_CAL_MAX_G) {
    Serial.printf(
        "[CALIB] WARN %s accel |a| %.4fg outside [0.8,1.2], scale=1.0\n",
        imu_name, accel_mean_g);
    return 1.0f;
  }
  return 1.0f / accel_mean_g;
}

static bool calibrate_bias() {
  float sd1[3], sd2[3];
  accel_scale1 = 1.0f;
  accel_scale2 = 1.0f;
  for (int a = 1; a <= BIAS_CALIB_RETRIES; a++) {
    Serial.printf("[CALIB] attempt %d/%d (hold still)...\n", a, BIAS_CALIB_RETRIES);
    float accel_mean1 = 0.0f, accel_mean2 = 0.0f;
    bool read1 = measure_imu_bias(
        IMU1, IMU1_SIGN, gyro_bias1, sd1, accel_mean1);
    bool read2 = measure_imu_bias(
        IMU2, IMU2_SIGN, gyro_bias2, sd2, accel_mean2);
    if (!read1 || !read2) {
      Serial.printf("[CALIB] sensor read failed (imu1=%d imu2=%d)\n", (int)read1, (int)read2);
      continue;
    }
    float max_sd = max(max(max(sd1[0],sd1[1]),sd1[2]),
                       max(max(sd2[0],sd2[1]),sd2[2]));
    Serial.printf("[CALIB] IMU1 %.3f %.3f %.3f | IMU2 %.3f %.3f %.3f (maxSD %.3f)\n",
                  gyro_bias1[0],gyro_bias1[1],gyro_bias1[2],
                  gyro_bias2[0],gyro_bias2[1],gyro_bias2[2], max_sd);
    if (max_sd <= BIAS_MOVEMENT_THRESH) {
      accel_scale1 = guarded_accel_scale(accel_mean1, "IMU1");
      accel_scale2 = guarded_accel_scale(accel_mean2, "IMU2");
      Serial.printf(
          "[CALIB] accel mean IMU1 %.4fg scale %.6f | "
          "IMU2 %.4fg scale %.6f\n",
          accel_mean1, accel_scale1, accel_mean2, accel_scale2);
      Serial.println("[CALIB] OK");
      return true;
    }
    Serial.println("[CALIB] movement detected, retry");
  }
  Serial.println("[CALIB] FAIL: reboot and calibrate on a stationary surface");
  return false;
}

// ==========================================================
// 7. 자세 추정용 적응 alpha (가속도 신뢰도 기반)
// ==========================================================
const float ACC_DEV_SOFT = 0.10f, ACC_DEV_HARD = 0.30f;
// 1kHz 루프 기준 시정수 tau ≈ dt/(1-alpha): 0.999→1s, 0.9995→2s, 0.9998→5s.
// 이전 값(0.99, tau 0.1s)은 250Hz legacy 루프에서 온 것으로, 1kHz에서는
// 비행 중 가속도(추력 방향+진동)에 자세 추정이 끌려가 호버가 불안정해진다.
const float ALPHA_STATIC = 0.999f, ALPHA_NORMAL = 0.9995f, ALPHA_DYN = 0.9998f;
static inline float compute_alpha(float ax, float ay, float az) {
  float dev = fabsf(sqrtf(ax*ax+ay*ay+az*az) - 1.0f);
  if (dev < ACC_DEV_SOFT) return ALPHA_STATIC;
  if (dev < ACC_DEV_HARD) return ALPHA_NORMAL;
  return ALPHA_DYN;
}

// 잠금 진입 시 1회만 실행되는 정리. 세 잠금 경로가 모두 이 함수를 통해서만
// wasLocked를 세운다. 매 tick 지우면 시동 해제 상태에서 `yaw 1`을 켤 수 없다.
static inline void enterLockedState(bool &wasLocked) {
  if (!wasLocked) yaw_hold_override = false;
  wasLocked = true;
}

// ==========================================================
// 8. PID 태스크 (Core 1, 1kHz)
// ==========================================================
void pid_task(void *pv) {
  // 모터 정지 수단(stopMotors, RC timeout 검사)이 모두 이 태스크 안에 있으므로,
  // SPI 행업 등으로 태스크가 블로킹되면 모터가 마지막 PWM으로 고정된다.
  // 태스크 워치독이 panic 재부팅을 강제하고, 재부팅 후 ESC는 PWM 신호
  // 소실로 정지하며 setup()의 stopMotors()가 정지 신호를 복원한다.
  esp_task_wdt_add(NULL);

  const TickType_t period = pdMS_TO_TICKS(1);   // vTaskDelayUntil 기반 (busy-wait 제거)
  TickType_t wake = xTaskGetTickCount();
  const float dt = 0.001f;

  LowPassFilter lpfD_Roll(40, dt), lpfD_Pitch(40, dt), lpfD_Yaw(40, dt);
  LowPassFilter lpfG1[3] = {{GYRO_SW_LPF_HZ, dt}, {GYRO_SW_LPF_HZ, dt}, {GYRO_SW_LPF_HZ, dt}};
  LowPassFilter lpfG2[3] = {{GYRO_SW_LPF_HZ, dt}, {GYRO_SW_LPF_HZ, dt}, {GYRO_SW_LPF_HZ, dt}};
  float prevGyroX = 0.0f, prevGyroY = 0.0f, prevGyroZ = 0.0f;

  FreezeMon fm1, fm2;
  uint32_t disagreeSince = 0;

  float targetRateRoll = 0, targetRatePitch = 0, targetRateYaw = 0;
  uint8_t outerCnt = 0;
  uint8_t magFusionCnt = 0;
  uint32_t lastMicros = micros();
  uint32_t loopCount = 0;
  uint32_t loopMarkerMs = millis();
  bool wasLocked = true;
  LandDetector landDet = {};
  uint32_t fs_enter_ms = 0;
  float fs_hold_yaw = 0.0f;

  inv_imu_sensor_event_t e1 = {}, e2 = {};

  while (true) {
    vTaskDelayUntil(&wake, period);
    esp_task_wdt_reset();
    TickType_t afterWake = xTaskGetTickCount();
    if ((TickType_t)(afterWake - wake) > period) {
      // 큰 지연 뒤 밀린 tick을 연속 실행하지 않는다(센서 stale read/D항 spike 방지).
      wake = afterWake;
    }
    uint32_t nowMs = millis();
    applyPendingSafetyRequest();
    loopCount++;
    uint32_t loopElapsedMs = nowMs - loopMarkerMs;
    if (loopElapsedMs >= 1000) {
      pidLoopHz = (int)((loopCount * 1000UL) / loopElapsedMs);
      loopCount = 0;
      loopMarkerMs = nowMs;
    }
    uint32_t nowUs = micros();
    float realDt = (nowUs - lastMicros) / 1e6f;
#if WIFI_LATENCY_DEBUG
    uint32_t rawDtUs = nowUs - lastMicros;
    if (rawDtUs > WIFI_LATENCY_PID_THRESHOLD_US) {
      recordLatEvent(pidLatRing, nowMs, rawDtUs, LAT_PID);
    }
#endif
    lastMicros = nowUs;
    // 지연 후 vTaskDelayUntil이 catch-up할 때 지나치게 작은 dt로 D항이 튀는 것을 방지.
    if (realDt < 0.0002f || realDt > 0.01f) realDt = dt;

    // ---------- 센서 읽기 ----------
    int readStatus1 = IMU1.getDataFromRegisters(e1);
    int readStatus2 = IMU2.getDataFromRegisters(e2);

    // 읽기 오류 시 event가 갱신되지 않으므로 동일 프레임으로 취급되어 freeze로 귀결된다.
    // fault는 비행 중 자동 복구하지 않고 다음 수동 start 때만 재평가/해제한다.
    bool frozen1 = checkFreeze(fm1, e1, nowMs);
    bool frozen2 = checkFreeze(fm2, e2, nowMs);
    imu1_frozen_now = frozen1;
    imu2_frozen_now = frozen2;
    if (frozen1 && !fault_imu1) Serial.printf("[FAULT] IMU1 FROZEN (read=%d)\n", readStatus1);
    if (frozen2 && !fault_imu2) Serial.printf("[FAULT] IMU2 FROZEN (read=%d)\n", readStatus2);
    if (frozen1) fault_imu1 = true;
    if (frozen2) fault_imu2 = true;

    // IMU1 sensor frame 기준 각속도 (IMU2 축 정렬 + 개별 bias 보정)
    float g1[3] = {
      e1.gyro[0]*GYRO_SCALE - gyro_bias1[0],
      e1.gyro[1]*GYRO_SCALE - gyro_bias1[1],
      e1.gyro[2]*GYRO_SCALE - gyro_bias1[2] };
    float g2[3] = {
      IMU2_SIGN_X*e2.gyro[0]*GYRO_SCALE - gyro_bias2[0],
      IMU2_SIGN_Y*e2.gyro[1]*GYRO_SCALE - gyro_bias2[1],
      IMU2_SIGN_Z*e2.gyro[2]*GYRO_SCALE - gyro_bias2[2] };
    // 소프트웨어 LPF. 제어와 IMU 불일치 판정 모두 필터된 각속도를 쓴다.
    // (진동으로 순간 차이가 15dps를 넘어 비행 중 disagree 컷이 나는 것을 방지)
    for (int k = 0; k < 3; k++) {
      g1[k] = lpfG1[k].update(g1[k]);
      g2[k] = lpfG2[k].update(g2[k]);
    }
    float a1[3] = {
      (float)e1.accel[0] * accel_scale1,
      (float)e1.accel[1] * accel_scale1,
      (float)e1.accel[2] * accel_scale1 };
    float a2[3] = {
      IMU2_SIGN_X*e2.accel[0] * accel_scale2,
      IMU2_SIGN_Y*e2.accel[1] * accel_scale2,
      IMU2_SIGN_Z*e2.accel[2] * accel_scale2 };

    // 현재 샘플 불일치는 재시동 거부 판단에도 사용한다.
    bool disagreeSample = false;
    if (!frozen1 && !frozen2) {
      for (int k = 0; k < 3; k++) {
        if (fabsf(g1[k] - g2[k]) > GYRO_DISAGREE_DPS) disagreeSample = true;
      }
    }
    imu_disagree_now = disagreeSample;

    // latch된 센서는 비행 중 값이 다시 움직여도 자동 재투입하지 않는다.
    bool h1 = !fault_imu1, h2 = !fault_imu2;

    // ---------- IMU 융합 (진짜 redundancy) ----------
    // 둘 다 정상: 불일치 검사 후 평균 / 하나만 정상: 그것만 / 중재 불가: fault
    if (h1 && h2) {
      if (disagreeSample) {
        if (disagreeSince == 0) disagreeSince = nowMs;
        else if (nowMs - disagreeSince >= IMU_DISAGREE_MS) {
          if (!fault_disagree) Serial.println("[FAULT] DUAL IMU DISAGREEMENT");
          fault_disagree = true;
        }
      } else {
        disagreeSince = 0;
      }
    } else {
      disagreeSince = 0;
    }

    float fg[3], fa[3];
    if (h1 && h2 && !fault_disagree) {
      for (int k = 0; k < 3; k++) { fg[k] = (g1[k]+g2[k])*0.5f; fa[k] = (a1[k]+a2[k])*0.5f; }
      active_imus = 2;
    } else if (h1 && !h2) {
      for (int k = 0; k < 3; k++) { fg[k] = g1[k]; fa[k] = a1[k]; }
      active_imus = 1;
    } else if (h2 && !h1) {
      for (int k = 0; k < 3; k++) { fg[k] = g2[k]; fa[k] = a2[k]; }
      active_imus = 1;
    } else {
      // 둘 다 freeze 또는 서로 안 맞음 -> 신뢰 불가
      active_imus = 0;
      if (fs_phase == FS_DESCENDING) fs_phase = FS_CUT_ABORT;
      safety_lock = true;
      mixer_scaled = false;
      iTermRoll = iTermPitch = iTermYaw = 0.0f;
      targetRateRoll = targetRatePitch = targetRateYaw = 0.0f;
      lpfD_Roll.reset(); lpfD_Pitch.reset(); lpfD_Yaw.reset();
      enterLockedState(wasLocked);
      motorOut[0] = 1000; motorOut[1] = 1000; motorOut[2] = 1000; motorOut[3] = 1000;
      tgtRate[0] = 0.0f; tgtRate[1] = 0.0f; tgtRate[2] = 0.0f;
      stopMotors();
      continue;
    }

    // Roll/pitch gyro now matches the validated accel transform; the inverted
    // signs caused estimator lag and rate-loop anti-damping.
    // bodyGz stays unchanged pending separate bench yaw re-verification.
    const float bodyGx =  fg[1];
    const float bodyGy = -fg[0];
    const float bodyGz = -fg[2];
    const float bodyAx =  fa[1] * ACCEL_SCALE;
    const float bodyAy = -fa[0] * ACCEL_SCALE;
    const float bodyAz =  fa[2] * ACCEL_SCALE;
    gyroX = bodyGx; gyroY = bodyGy; gyroZ = bodyGz;
    accX = bodyAx; accY = bodyAy; accZ = bodyAz;

    // ---------- 자세 추정 (상보필터) ----------
    angleX += bodyGx * realDt;
    angleY += bodyGy * realDt;
    if (fabsf(bodyGz) > YAW_DEADZONE) angleZ += bodyGz * realDt;

    float alpha = compute_alpha(bodyAx, bodyAy, bodyAz);
    float accAngleX = atan2f(bodyAy, sqrtf(bodyAx*bodyAx + bodyAz*bodyAz)) * 180.0f / PI;
    float accAngleY = atan2f(-bodyAx, sqrtf(bodyAy*bodyAy + bodyAz*bodyAz)) * 180.0f / PI;
    angleX = alpha * angleX + (1.0f - alpha) * accAngleX;
    angleY = alpha * angleY + (1.0f - alpha) * accAngleY;

    // outerCnt는 safety-lock 경로에서 매 tick reset되므로 독립 divider를 쓴다.
    // 이로써 armed/disarmed 모두 정확히 250Hz, K_MAG tau≈4s를 유지한다.
    if (magFusionCnt == 0 && mag_enabled) {
      MagSnapshot mag;
      if (readMagSnapshot(mag)
          // 코어0 갱신이 코어1의 nowMs보다 미래일 수 있어 부호 있는 차이로 비교한다.
          && (int32_t)(nowMs - mag.sample_ms) <= (int32_t)MAG_STALE_MS
          && mag.x*mag.x + mag.y*mag.y + mag.z*mag.z > 1e-6f) {
        const float magThrDelta = (float)(base_throttle - MAG_THROTTLE_REF_US);
        mag.x -= mag_comp_x * magThrDelta;
        mag.y -= mag_comp_y * magThrDelta;
        mag.z -= mag_comp_z * magThrDelta;
        magTelemX = mag.x; magTelemY = mag.y; magTelemZ = mag.z;  // 보정 적용값(=k 0일 때 raw)
        float heading = tiltCompensatedMagHeadingDeg(
            mag.x, mag.y, mag.z, angleX, angleY);
        if (isfinite(heading)) {
          magHeading = heading;
          if (mag_reference_pending) {
            // Relative heading: enabling mag or resetting angleZ must not
            // command a turn toward magnetic north.
            magYawReferenceOffset = wrapDeg(angleZ - heading);
            mag_reference_pending = false;
          }
          float referencedHeading =
              wrapDeg(heading + magYawReferenceOffset);
          angleZ += magYawCorrectionDeg(angleZ, referencedHeading, K_MAG);
        }
      }
    }
    magFusionCnt++;
    if (magFusionCnt >= OUTER_DIV) magFusionCnt = 0;

    // ---------- 안전 검사 ----------
    if (fabsf(angleX) > SAFETY_ANGLE || fabsf(angleY) > SAFETY_ANGLE) {
      if (!fault_attitude) Serial.printf("[FAULT] OVER-TILT R:%.1f P:%.1f\n", angleX, angleY);
      fault_attitude = true;
      safety_lock = true;
    }
    if (takeFailsafeResumeRequest()) {
      const ResumeRefusalReason refusal = resumeRefusalReason(nowMs);
      if (refusal != RESUME_ALLOWED) {
        logResumeRefusal(refusal);
      } else {
        fs_phase = FS_NONE;
        fault_rc = false;
        base_throttle = (int)lroundf(hover_est);
        min_throttle = max(1050, base_throttle - CTRL_MARGIN);
        max_throttle = min(1900, base_throttle + CTRL_MARGIN);
        Serial.printf(">>> RESUME hover=%d\n", base_throttle);
      }
    }
    // 잠금 해제 첫 tick의 D kick과 이전 목표 rate 잔류를 제거한다.
    // 이 블록은 RC timeout 진입 가드보다 먼저 실행해야 한다. 뒤에 두면 같은
    // tick에서 새로 진입한 FS_DESCENDING까지 FS_NONE으로 덮어쓰게 된다.
    if (wasLocked && !safety_lock) {
      // start가 남긴 이전 비행의 terminal phase와 start/abort 경합 결과를
      // Core 1에서 해제해 fs_phase의 런타임 writer를 pid_task 하나로 유지한다.
      fs_phase = FS_NONE;
      prevGyroX = bodyGx; prevGyroY = bodyGy; prevGyroZ = bodyGz;
      lpfD_Roll.reset(); lpfD_Pitch.reset(); lpfD_Yaw.reset();
      targetRateRoll = targetRatePitch = targetRateYaw = 0.0f;
      outerCnt = 0;
      wasLocked = false;
    }

    const float accelMag =
        sqrtf(accX*accX + accY*accY + accZ*accZ);
    const bool hoverSampleEligible =
        !safety_lock && fs_phase == FS_NONE &&
        fabsf(angleX) < HOVER_LEVEL_MAX_DEG &&
        fabsf(angleY) < HOVER_LEVEL_MAX_DEG &&
        fabsf(accelMag - 1.0f) <= HOVER_ACCEL_TOL_G &&
        base_throttle > HOVER_MIN_THROTTLE_US;
    updateHoverThrottleEstimator(
        hoverTracker, hoverSampleEligible, base_throttle, nowMs, realDt,
        HOVER_LPF_TAU_S, HOVER_VALID_MS);
    hover_est = hoverTracker.estimate_us;
    hover_valid = hoverTracker.valid;

    // RC 타임아웃은 유효 호버 추정치가 없거나 호버 대비 지상 스로틀이면
    // 즉시 컷하고, 그 외에는 추정 호버를 기준으로 자동착륙한다.
    // 진입 블록 전체를 fs_phase 가드 안에 둔다 — 가드가 없으면 하강 내내
    // rcTimedOut이 참이라 이 블록이 1kHz로 재실행되고, Serial.println이 TX
    // 버퍼를 포화시켜 pid_task를 블로킹하면 500ms 태스크 워치독이 비행 중
    // 재부팅을 일으킨다.
    if (fs_phase == FS_NONE && !safety_lock && rcTimedOut(nowMs, lastRcMs)) {
      fault_rc = true;
      if (!hover_valid) {
        safety_lock = true;
        Serial.println("[FAULT] RC TIMEOUT -> CUT (NO HOVER EST)");  // 한 번만
      } else if ((float)base_throttle <
                 hover_est - (float)FS_GROUND_MARGIN_US) {
        safety_lock = true;
        Serial.println("[FAULT] RC TIMEOUT -> GROUND CUT");  // 한 번만
      } else {
        fs_phase = FS_DESCENDING;
        fs_hold_yaw = angleZ;
        fs_enter_ms = nowMs;
        landDet = {};
        fs_probe_state = FS_PROBE_WAIT;
        fs_probe_no_response = 0;
        fs_probe_response_g = 0.0f;
        base_throttle = failsafeDescentThrottle(
            (int)lroundf(hover_est), FS_DESCENT_DELTA_US);
        Serial.println("[FAULT] RC TIMEOUT -> AUTO-LAND");   // 한 번만
      }
    }

    // 하강 중에는 목표를 매 tick 덮어쓴다. 링크가 돌아와도 rc/rcr 파서가 쓴
    // 값이 무해해지므로 파서 쪽에 failsafe 분기를 넣을 필요가 없다.
    if (fs_phase == FS_DESCENDING) {
      targetAngleX = trim_roll;
      targetAngleY = trim_pitch;
      targetAngleZ = fs_hold_yaw;
      targetYawRate = 0.0f;
      const int descentThrottle = failsafeDescentThrottle(
          (int)lroundf(hover_est), FS_DESCENT_DELTA_US);

      const uint32_t elapsed = nowMs - fs_enter_ms;
      const bool landed = updateLandDetector(
          landDet, accelMag, elapsed, descentThrottle,
          FS_LAND_PROBE_CONFIG);
      fs_probe_state = landDet.probe_state;
      fs_probe_no_response = landDet.no_response_count;
      fs_probe_response_g = landDet.last_response_g;

      // collective 창은 정상 하강값에 고정한다. 40us 딥은 CTRL_MARGIN=150us
      // 안에서 base만 낮추므로 자세 mixer의 기존 authority를 보존한다.
      base_throttle =
          descentThrottle -
          (landDetectorProbeActive(landDet) ? FS_PROBE_DIP_US : 0);
      min_throttle = max(1050, descentThrottle - CTRL_MARGIN);
      max_throttle = min(1900, descentThrottle + CTRL_MARGIN);
      const FailsafePhase next = failsafeStep(landed, elapsed, FS_MAX_MS);
      if (next != FS_DESCENDING) {
        fs_phase = next;
        safety_lock = true;      // 아래 safety_lock 블록이 이번 tick에 정리한다
        Serial.printf(">>> AUTO-LAND END phase=%d\n", (int)next);
      }
    }
    if (safety_lock) {
      if (fs_phase == FS_DESCENDING) fs_phase = FS_CUT_ABORT;
      mixer_scaled = false;
      iTermRoll = iTermPitch = iTermYaw = 0.0f;
      targetRateRoll = targetRatePitch = targetRateYaw = 0.0f;
      targetYawRate = 0.0f;
      targetAngleZ = angleZ;
      yaw_hold_now = false;
      prevGyroX = bodyGx; prevGyroY = bodyGy; prevGyroZ = bodyGz;
      lpfD_Roll.reset(); lpfD_Pitch.reset(); lpfD_Yaw.reset();
      outerCnt = 0;
      enterLockedState(wasLocked);
      motorOut[0] = 1000; motorOut[1] = 1000; motorOut[2] = 1000; motorOut[3] = 1000;
      tgtRate[0] = 0.0f; tgtRate[1] = 0.0f; tgtRate[2] = 0.0f;
      stopMotors();
      continue;
    }

    // ---------- Outer loop (250Hz): 각도 -> 목표 각속도 ----------
    // yaw는 각속도 명령 + 자동 heading 잠금. hold가 아닌 동안 setpoint를
    // 매 tick 현재 heading으로 슬레이빙해 stale setpoint를 원천 차단한다.
    const YawOuter yawOuter = updateYawOuter(
        targetYawRate, bodyGz, angleZ, targetAngleZ,
        yaw_hold_override, YAW_RATE_DEADZONE, YAW_HOLD_SETTLE_DPS);
    targetAngleZ = yawOuter.target_angle_deg;
    yaw_hold_now = yawOuter.hold;

    if (outerCnt == 0) {
      targetRateRoll = constrain((targetAngleX - angleX) * Kp_Angle_Roll,
                                 -MAX_TARGET_RATE_RP, MAX_TARGET_RATE_RP);
      targetRatePitch = constrain((targetAngleY - angleY) * Kp_Angle_Pitch,
                                  -MAX_TARGET_RATE_RP, MAX_TARGET_RATE_RP);
      targetRateYaw = yawTargetRateDps(yawOuter, targetYawRate, angleZ,
                                       Kp_Angle_Yaw, MAX_TARGET_RATE_YAW);
    }
    outerCnt++;
    if (outerCnt >= OUTER_DIV) outerCnt = 0;

    // ---------- Inner loop (1kHz): 각속도 PID ----------
    float eRoll  = targetRateRoll  - bodyGx;
    float ePitch = targetRatePitch - bodyGy;
    float eYaw   = targetRateYaw   - bodyGz;

    // D는 gyro 미분에만 건다 (setpoint/outer loop 오염 제거)
    float dRoll  = lpfD_Roll.update((bodyGx - prevGyroX) / realDt);
    float dPitch = lpfD_Pitch.update((bodyGy - prevGyroY) / realDt);
    float dYaw   = lpfD_Yaw.update((bodyGz - prevGyroZ) / realDt);
    prevGyroX = bodyGx; prevGyroY = bodyGy; prevGyroZ = bodyGz;

    float pidRoll  = Kp_Rate_Roll *eRoll  + iTermRoll  - Kd_Rate_Roll *dRoll;
    float pidPitch = Kp_Rate_Pitch*ePitch + iTermPitch - Kd_Rate_Pitch*dPitch;
    // yaw 각도 유지가 꺼져 있어도 rate 감쇠(target 0)는 항상 동작한다.
    // 모터 토크 불균형과 롤·피치 보정의 반작용으로 생기는 자유 회전을 억제한다.
    float pidYaw   = Kp_Rate_Yaw*eYaw + iTermYaw - Kd_Rate_Yaw*dYaw;

    // ---------- 모터 desaturation + 포화 기반 anti-windup ----------
    const int throttle = base_throttle;
    MotorMix mix = mixAndDesaturate(pidRoll, pidPitch, pidYaw,
                                    throttle, min_throttle, max_throttle);
    mixer_scaled = mix.scaled;

    // scale은 자세 명령이 실제로 잘린 경우다. collective 이동만 일어난 경우에는
    // 자세 authority가 보존되므로 적분을 계속한다.
    // 적분은 모터 출력 기여(µs) 단위로 누적하고 I_TERM_MAX_US로 제한한다.
    // (이전 errorSum ±200deg 클램프는 기본 Ki에서 최대 기여 1µs라 적분이
    //  사실상 없는 것과 같아 CG 치우침 같은 정상상태 오차를 못 잡았다.)
    if (throttle > 1100 && !mix.scaled) {
      iTermRoll  = constrain(iTermRoll  + Ki_Rate_Roll  * eRoll  * realDt,
                             -I_TERM_MAX_US, I_TERM_MAX_US);
      iTermPitch = constrain(iTermPitch + Ki_Rate_Pitch * ePitch * realDt,
                             -I_TERM_MAX_US, I_TERM_MAX_US);
      // yaw 적분은 rate/hold 두 모드 모두에서 돈다. 안쪽 루프의 임무가
      // "목표 각속도 추종"이고, P 단독으로는 모터 토크 불균형 같은 일정
      // 외란에서 정상상태 각속도 오차가 남아 정착 임계치 아래로 내려가지
      // 않는다(2026-07-27 실측 최대 +16.6dps). 그러면 자동 잠금이 영영
      // 걸리지 않는다.
      iTermYaw = constrain(iTermYaw + Ki_Rate_Yaw * eYaw * realDt,
                           -I_TERM_MAX_US, I_TERM_MAX_US);
    } else if (throttle <= 1100) {
      iTermRoll = iTermPitch = iTermYaw = 0.0f;
    }

    // 모터 PWM의 단일 writer는 PID task로 유지한다.
    if (safety_lock) {
      motorOut[0] = 1000; motorOut[1] = 1000; motorOut[2] = 1000; motorOut[3] = 1000;
      tgtRate[0] = 0.0f; tgtRate[1] = 0.0f; tgtRate[2] = 0.0f;
      stopMotors();
      enterLockedState(wasLocked);
      continue;
    }
    writeMotor(pinM1, mix.motor[0]);
    writeMotor(pinM2, mix.motor[1]);
    writeMotor(pinM3, mix.motor[2]);
    writeMotor(pinM4, mix.motor[3]);
    motorOut[0] = mix.motor[0]; motorOut[1] = mix.motor[1];
    motorOut[2] = mix.motor[2]; motorOut[3] = mix.motor[3];
    tgtRate[0] = targetRateRoll; tgtRate[1] = targetRatePitch; tgtRate[2] = targetRateYaw;
  }
}

// ==========================================================
// 9. UDP 태스크 (Core 0) — char* 파싱 (String heap 회피)
// ==========================================================
static char *trimCommand(char *s) {
  while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') s++;
  char *end = s + strlen(s);
  while (end > s && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r' || end[-1] == '\n')) end--;
  *end = '\0';
  return s;
}

static bool parseFloatStrict(const char *text, float &out) {
  char *end;
  float value = strtof(text, &end);
  if (end == text || !isfinite(value)) return false;
  while (*end == ' ' || *end == '\t') end++;
  if (*end != '\0') return false;
  out = value;
  return true;
}

static bool parseIntStrict(const char *text, long &out) {
  char *end;
  long value = strtol(text, &end, 10);
  if (end == text) return false;
  while (*end == ' ' || *end == '\t') end++;
  if (*end != '\0') return false;
  out = value;
  return true;
}

static bool setRcTargets(float x, float y, float z, bool hasYaw) {
  if (!isfinite(x) || !isfinite(y) || (hasYaw && !isfinite(z))) return false;
  // 트림을 더한 뒤 클램프해야 총합이 ±30°로 제한된다.
  targetAngleX = constrain(x + trim_roll,  -MAX_TARGET_ANGLE_RP, MAX_TARGET_ANGLE_RP);
  targetAngleY = constrain(y + trim_pitch, -MAX_TARGET_ANGLE_RP, MAX_TARGET_ANGLE_RP);
  if (hasYaw) targetAngleZ = z;
  lastRcMs = millis();
  return true;
}

// 표준 형식: rc <seq> <roll> <pitch> <yaw>
// 벤치 수동 테스트용으로 rc <roll> <pitch> [yaw]도 허용한다.
static void handleRcCommand(char *buf) {
  char *save = nullptr;
  char *token = strtok_r(buf, " \t", &save); // "rc"
  (void)token;
  char *arg[4] = {nullptr, nullptr, nullptr, nullptr};
  int count = 0;
  while (count < 4 && (arg[count] = strtok_r(nullptr, " \t", &save)) != nullptr) count++;
  if (strtok_r(nullptr, " \t", &save) != nullptr) return; // 여분 필드 거부

  if (count == 4) {
    if (arg[0][0] == '-' || arg[0][0] == '+') return;
    char *seqEnd;
    errno = 0;
    unsigned long seqLong = strtoul(arg[0], &seqEnd, 10);
    if (seqEnd == arg[0] || *seqEnd != '\0' || errno == ERANGE ||
        seqLong > 0xFFFFFFFFUL) return;

    float x, y, z;
    if (!parseFloatStrict(arg[1], x) || !parseFloatStrict(arg[2], y) ||
        !parseFloatStrict(arg[3], z)) return;

    uint32_t seq = (uint32_t)seqLong;
    rcTotalPkts = rcTotalPkts + 1;
    if (rcSeqValid) {
      int32_t advance = (int32_t)(seq - lastRcSeq); // uint32 wrap도 정상 처리
      if (advance <= 0) {
        rcDroppedPkts = rcDroppedPkts + 1;
        return; // 지연 도착/중복 패킷 폐기
      }
      if (advance > 1) rcDroppedPkts += (uint32_t)(advance - 1);
    }
    lastRcSeq = seq;
    rcSeqValid = true;
    setRcTargets(x, y, z, true);
    return;
  }

  if (count == 2 || count == 3) {
    float x, y, z = 0.0f;
    if (!parseFloatStrict(arg[0], x) || !parseFloatStrict(arg[1], y)) return;
    if (count == 3 && !parseFloatStrict(arg[2], z)) return;
    setRcTargets(x, y, z, count == 3);
  }
}

// rcr <seq> <roll> <pitch> <yaw_rate>
// yaw는 각속도(dps) 명령이다. seq 처리와 워치독 급이는 rc와 상태를 공유한다.
static void handleRcrCommand(char *buf) {
  char *save = nullptr;
  (void)strtok_r(buf, " \t", &save); // "rcr"
  char *arg[4] = {nullptr, nullptr, nullptr, nullptr};
  int count = 0;
  while (count < 4 && (arg[count] = strtok_r(nullptr, " \t", &save)) != nullptr) count++;
  if (count != 4) return;
  if (strtok_r(nullptr, " \t", &save) != nullptr) return; // 여분 필드 거부

  if (arg[0][0] == '-' || arg[0][0] == '+') return;
  char *seqEnd;
  errno = 0;
  unsigned long seqLong = strtoul(arg[0], &seqEnd, 10);
  if (seqEnd == arg[0] || *seqEnd != '\0' || errno == ERANGE ||
      seqLong > 0xFFFFFFFFUL) return;

  float x, y, rate;
  if (!parseFloatStrict(arg[1], x) || !parseFloatStrict(arg[2], y) ||
      !parseFloatStrict(arg[3], rate)) return;

  uint32_t seq = (uint32_t)seqLong;
  rcTotalPkts = rcTotalPkts + 1;
  if (rcSeqValid) {
    int32_t advance = (int32_t)(seq - lastRcSeq);
    if (advance <= 0) {
      rcDroppedPkts = rcDroppedPkts + 1;
      return;
    }
    if (advance > 1) rcDroppedPkts += (uint32_t)(advance - 1);
  }
  lastRcSeq = seq;
  rcSeqValid = true;

  if (!setRcTargets(x, y, 0.0f, false)) return;   // roll/pitch만, yaw 각도는 건드리지 않는다
  targetYawRate = constrain(rate, -MAX_TARGET_RATE_YAW, MAX_TARGET_RATE_YAW);
}

static void handleGainCommand(const char *buf) {
  if (strlen(buf) < 3) return;
  float value;
  if (!parseFloatStrict(buf + 2, value) || value < 0.0f || value > 100.0f) return;

  // Cascade 전용 공통 명령
  if      (strncmp(buf, "rp", 2) == 0) { Kp_Rate_Roll = value; Kp_Rate_Pitch = value; }
  else if (strncmp(buf, "ri", 2) == 0) { Ki_Rate_Roll = value; Ki_Rate_Pitch = value; }
  else if (strncmp(buf, "rd", 2) == 0) { Kd_Rate_Roll = value; Kd_Rate_Pitch = value; }
  else if (strncmp(buf, "ap", 2) == 0) { Kp_Angle_Roll = value; Kp_Angle_Pitch = value; }
  else if (strncmp(buf, "ar", 2) == 0) Kp_Angle_Roll = value;
  else if (strncmp(buf, "at", 2) == 0) Kp_Angle_Pitch = value;
  else if (strncmp(buf, "ay", 2) == 0) Kp_Angle_Yaw = value;
  else if (strncmp(buf, "yp", 2) == 0) Kp_Rate_Yaw = value;
  else if (strncmp(buf, "yi", 2) == 0) Ki_Rate_Yaw = value;
  else if (strncmp(buf, "yd", 2) == 0) Kd_Rate_Yaw = value;

  // scripts/tune_pid.py 명령 호환: P/I/D는 inner rate PID에 대응
  else if (strncmp(buf, "pa", 2) == 0) { Kp_Rate_Roll = value; Kp_Rate_Pitch = value; }
  else if (strncmp(buf, "ia", 2) == 0) { Ki_Rate_Roll = value; Ki_Rate_Pitch = value; }
  else if (strncmp(buf, "da", 2) == 0) { Kd_Rate_Roll = value; Kd_Rate_Pitch = value; }
  else if (strncmp(buf, "pr", 2) == 0) Kp_Rate_Roll = value;
  else if (strncmp(buf, "ir", 2) == 0) Ki_Rate_Roll = value;
  else if (strncmp(buf, "dr", 2) == 0) Kd_Rate_Roll = value;
  else if (strncmp(buf, "pp", 2) == 0) Kp_Rate_Pitch = value;
  else if (strncmp(buf, "ip", 2) == 0) Ki_Rate_Pitch = value;
  else if (strncmp(buf, "dp", 2) == 0) Kd_Rate_Pitch = value;
  else if (strncmp(buf, "py", 2) == 0) Kp_Rate_Yaw = value;
  else if (strncmp(buf, "iy", 2) == 0) Ki_Rate_Yaw = value;
  else if (strncmp(buf, "dy", 2) == 0) Kd_Rate_Yaw = value;
}

// 첫 14개 필드는 기존 PC 스크립트와 호환된다. 뒤 필드는 cascade 진단 확장:
// fault_imu1,fault_imu2,fault_disagree,active_imus,scaled,fault_attitude,
// calibration_ok,armed. 그 뒤 Tier1 8개와 MagHeading, Mag_X/Y/Z, Yaw_Hold,
// Failsafe_Phase, Trim_Roll/Pitch, Hover_Est/Valid와 프로브 상태/연속
// 무반응/최근 차분 반응을 append-only로 보낸다.
static void sendTelemetry() {
  if (!connectionEstablished) return;
  bool criticalFault = (active_imus == 0) || fault_attitude || !calibration_ok;
  const float hoverEstSnapshot = hover_est;
  const bool hoverValidSnapshot = hover_valid;
  const uint8_t probeStateSnapshot = fs_probe_state;
  const uint8_t probeNoResponseSnapshot = fs_probe_no_response;
  const float probeResponseSnapshot = fs_probe_response_g;
  udp.beginPacket(laptopIP, laptopPort);
  udp.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%d,%d,%d,%lu,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%.2f,%.2f,%.2f,%d,%d,%d,%.3f",
             angleX, angleY, angleZ,
             gyroX, gyroY, gyroZ,
             accX, accY, accZ,
             base_throttle,
             (int)fault_rc, (int)criticalFault,
             (unsigned long)rcTotalPkts, (unsigned long)rcDroppedPkts,
             (int)fault_imu1, (int)fault_imu2, (int)fault_disagree,
             active_imus, (int)mixer_scaled, (int)fault_attitude,
             (int)calibration_ok, (int)!safety_lock,
             motorOut[0], motorOut[1], motorOut[2], motorOut[3], pidLoopHz,
             tgtRate[0], tgtRate[1], tgtRate[2], magHeading,
             magTelemX, magTelemY, magTelemZ, (int)yaw_hold_now,
             (int)fs_phase, trim_roll, trim_pitch,
             hoverEstSnapshot, (int)hoverValidSnapshot,
             (int)probeStateSnapshot, (int)probeNoResponseSnapshot,
             probeResponseSnapshot);
  udp.endPacket();
}

void udp_task(void *pv) {
  uint32_t lastSend = 0;
#if WIFI_LATENCY_DEBUG
  uint32_t prevTopUs = micros();
#endif
  while (true) {
#if WIFI_LATENCY_DEBUG
    uint32_t topUs = micros();
    uint32_t loopGapUs = topUs - prevTopUs;
    prevTopUs = topUs;
    if (loopGapUs > WIFI_LATENCY_UDP_THRESHOLD_US) {
      recordLatEvent(udpLatRing, millis(), loopGapUs, LAT_UDP_LOOP);
    }
    uint32_t parseStartUs = micros();
#endif
    int packetSize = udp.parsePacket();
#if WIFI_LATENCY_DEBUG
    uint32_t parseDurUs = micros() - parseStartUs;
    if (parseDurUs > WIFI_LATENCY_UDP_THRESHOLD_US) {
      recordLatEvent(udpLatRing, millis(), parseDurUs, LAT_UDP_PARSE);
    }
#endif
    if (packetSize) {
      laptopIP = udp.remoteIP();
      laptopPort = udp.remotePort();
      connectionEstablished = true;

      int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1); // off-by-one 방지
      if (len > 0) {
        packetBuffer[len] = '\0';
        char *buf = trimCommand(packetBuffer);

        if (strncmp(buf, "rcr", 3) == 0 && (buf[3] == ' ' || buf[3] == '\t')) {
          handleRcrCommand(buf);
        }
        else if (strncmp(buf, "rc", 2) == 0 && (buf[2] == ' ' || buf[2] == '\t')) {
          handleRcCommand(buf);
        }
        else if (strcmp(buf, "start") == 0) {
          bool overTilt = fabsf(angleX) > SAFETY_ANGLE || fabsf(angleY) > SAFETY_ANGLE;
          bool noUsableImu = imu1_frozen_now && imu2_frozen_now;
          if (!safety_lock) {
            // 이미 시동 상태. 지상국은 start를 여러 번 재전송하므로, 지연
            // 도착한 중복 start가 비행 중 fault latch를 지우고 스로틀 창을
            // 리셋하는 것을 막는다.
            Serial.println(">>> START ignored (already armed)");
          }
          else if (!calibration_ok || overTilt || noUsableImu
                   || imu_disagree_now || mag_calibrating) {
            Serial.printf(">>> START REFUSED calib=%d tilt=%d imu=%d disagree=%d magcal=%d\n",
                          (int)calibration_ok, (int)overTilt,
                          (int)noUsableImu, (int)imu_disagree_now,
                          (int)mag_calibrating);
          } else {
            // safety_lock=true인 동안 나머지 비행 상태는 Core 0에서 초기화한다.
            fault_rc = false;
            fault_imu1 = imu1_frozen_now;
            fault_imu2 = imu2_frozen_now;
            fault_disagree = false;
            fault_attitude = false;
            lastRcSeq = 0;
            rcSeqValid = false;
            lastRcMs = millis();
            base_throttle = 1100; min_throttle = 1050; max_throttle = 1250;
            targetAngleX = 0.0f;
            targetAngleY = 0.0f;
            targetAngleZ = 0.0f;
            angleZ = 0.0f;
            mag_reference_pending = true;
            hoverTracker = {};
            hover_est = 0.0f;
            hover_valid = false;
            fs_probe_state = FS_PROBE_WAIT;
            fs_probe_no_response = 0;
            fs_probe_response_g = 0.0f;
            requestSafetyArm();
            Serial.println(">>> START");
          }
        }
        else if (strcmp(buf, "stop") == 0) {
          requestSafetyDisarm();
          base_throttle = 1000;
          Serial.println(">>> STOP");
        }
        else if (strcmp(buf, "resume") == 0) {
          const ResumeRefusalReason refusal = resumeRefusalReason(millis());
          if (refusal != RESUME_ALLOWED) {
            logResumeRefusal(refusal);
          } else {
            requestFailsafeResume();
          }
        }
        else if (strncmp(buf, "th", 2) == 0) {
          long parsed;
          if (parseIntStrict(buf + 2, parsed)) {
            int nb = constrain((int)parsed, 1000, 1900);
            base_throttle = nb;
            min_throttle = max(1050, nb - CTRL_MARGIN);
            max_throttle = min(1900, nb + CTRL_MARGIN);
          }
        }
        else if (strncmp(buf, "yaw", 3) == 0) {
          long enabled;
          if (parseIntStrict(buf + 3, enabled) && (enabled == 0 || enabled == 1)) {
            if (enabled == 0) {
              yaw_hold_override = false;
              Serial.println(">>> Yaw hold OFF (auto)");
            } else if (!safety_lock) {
              // 비행 중 임의로 켜면 지상국 setpoint와 어긋난 채 최대 권한
              // 슬램이 날 수 있다. 시동 해제 상태에서만 켤 수 있게 한다.
              Serial.println(">>> Yaw hold refused (armed)");
            } else {
              targetAngleZ = angleZ;
              yaw_hold_override = true;
              Serial.println(">>> Yaw hold ON (override)");
            }
          }
        }
        else if (strncmp(buf, "magcal", 6) == 0) {
          long enabled;
          if (parseIntStrict(buf + 6, enabled) && (enabled == 0 || enabled == 1)) {
            if (enabled == 0) {
              stopMagCalibration();
            } else if (!safety_lock) {
              Serial.println(">>> Magcal refused (armed)");
            } else if (initMagnetometer()) {
              startMagCalibration();
            }
          }
        }
        else if (strncmp(buf, "magc", 4) == 0) {
          char *save = nullptr;
          char *token = strtok_r(buf, " \t", &save); // "magc"
          (void)token;
          char *arg[3] = {nullptr, nullptr, nullptr};
          int count = 0;
          while (count < 3 && (arg[count] = strtok_r(nullptr, " \t", &save)) != nullptr) count++;
          if (count == 3 && strtok_r(nullptr, " \t", &save) == nullptr) {
            float x, y, z;
            if (parseFloatStrict(arg[0], x) && parseFloatStrict(arg[1], y) &&
                parseFloatStrict(arg[2], z)) {
              mag_comp_x = x;
              mag_comp_y = y;
              mag_comp_z = z;
              Serial.printf(">>> mag comp = %.4f %.4f %.4f uT/us\n",
                            (double)mag_comp_x, (double)mag_comp_y, (double)mag_comp_z);
            }
          }
        }
        else if (strncmp(buf, "mag", 3) == 0) {
          long enabled;
          if (parseIntStrict(buf + 3, enabled) && (enabled == 0 || enabled == 1)) {
            if (enabled == 0) {
              mag_enabled = false;
              mag_reference_pending = true;
              Serial.println(">>> Mag OFF");
            } else if (mag_calibrating) {
              Serial.println(">>> Mag refused (magcal active)");
            } else if (!mag_ready && !safety_lock) {
              // First-time init does blocking I2C/delay(100); only when disarmed.
              Serial.println(">>> Mag refused (armed, not initialized)");
            } else if (initMagnetometer()) {
              mag_reference_pending = true;
              mag_enabled = true;
              Serial.println(">>> Mag ON");
            }
          }
        }
        else if (strncmp(buf, "trim", 4) == 0 &&
                 (buf[4] == ' ' || buf[4] == '\t')) {
          char *save = nullptr;
          (void)strtok_r(buf, " \t", &save);          // "trim"
          char *arg0 = strtok_r(nullptr, " \t", &save);
          char *arg1 = strtok_r(nullptr, " \t", &save);
          if (arg0 != nullptr && arg1 != nullptr &&
              strtok_r(nullptr, " \t", &save) == nullptr) {
            float r, p;
            if (parseFloatStrict(arg0, r) && parseFloatStrict(arg1, p)) {
              trim_roll  = constrain(r, -TRIM_MAX_DEG, TRIM_MAX_DEG);
              trim_pitch = constrain(p, -TRIM_MAX_DEG, TRIM_MAX_DEG);
              Serial.printf(">>> Trim R:%.2f P:%.2f\n", trim_roll, trim_pitch);
            }
          }
        }
        else if (strcmp(buf, "gains") == 0) {
          if (connectionEstablished) {
            udp.beginPacket(laptopIP, laptopPort);
            udp.printf("GAINS,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
                       Kp_Angle_Roll, Kp_Angle_Pitch, Kp_Angle_Yaw,
                       Kp_Rate_Roll, Kp_Rate_Pitch, Kp_Rate_Yaw,
                       Ki_Rate_Roll, Ki_Rate_Pitch, Ki_Rate_Yaw,
                       Kd_Rate_Roll, Kd_Rate_Pitch, Kd_Rate_Yaw);
            udp.endPacket();
          }
        }
        else {
          handleGainCommand(buf);
        }
      }
    }

    uint32_t now = millis();
    if (mag_enabled || mag_calibrating) sampleMagnetometer(now);
    if (now - lastSend >= 50) {
      lastSend = now;
#if WIFI_LATENCY_DEBUG
      uint32_t sendStartUs = micros();
#endif
      sendTelemetry();
#if WIFI_LATENCY_DEBUG
      uint32_t sendDurUs = micros() - sendStartUs;
      if (sendDurUs > WIFI_LATENCY_UDP_THRESHOLD_US) {
        recordLatEvent(udpLatRing, millis(), sendDurUs, LAT_UDP_SEND);
      }
#endif
    }
    vTaskDelay(1);
  }
}

// ==========================================================
// 10. setup / loop
// ==========================================================
void setup() {
  Serial.begin(115200);

  pinMode(SPI_CS1, OUTPUT); pinMode(SPI_CS2, OUTPUT);
  digitalWrite(SPI_CS1, HIGH); digitalWrite(SPI_CS2, HIGH);   // float CS 로 인한 버스 경합 방지
  delay(100);

  WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  udp.begin(UDP_PORT);

  SPI.begin(12, 13, 11, -1);   // SCK, MISO, MOSI; CS는 각 IMU 객체가 관리

  bool esc_ok = ledcAttach(pinM1, ESC_FREQ, ESC_RES)
             && ledcAttach(pinM2, ESC_FREQ, ESC_RES)
             && ledcAttach(pinM3, ESC_FREQ, ESC_RES)
             && ledcAttach(pinM4, ESC_FREQ, ESC_RES);
  if (!esc_ok) while (1) { Serial.println("[FAULT] ESC attach FAIL"); delay(1000); }
  stopMotors();

  if (IMU1.begin() < 0) while (1) { Serial.println("[FAULT] IMU1 FAIL"); delay(1000); }
  if (IMU2.begin() < 0) while (1) { Serial.println("[FAULT] IMU2 FAIL"); delay(1000); }
  int sensorStartStatus = 0;
  sensorStartStatus |= IMU1.startAccel(1600, 16);
  sensorStartStatus |= IMU1.startGyro(1600, 2000);
  sensorStartStatus |= IMU2.startAccel(1600, 16);
  sensorStartStatus |= IMU2.startGyro(1600, 2000);
  sensorStartStatus |= IMU1.setLowPassFilters();
  sensorStartStatus |= IMU2.setLowPassFilters();
  if (sensorStartStatus != 0) {
    while (1) { Serial.printf("[FAULT] IMU START FAIL (%d)\n", sensorStartStatus); delay(1000); }
  }
  delay(500);

  calibration_ok = calibrate_bias();

  // pid_task 전용 태스크 워치독. idle task는 감시하지 않고, timeout 시
  // panic 재부팅으로 마지막 PWM 고정 상태에서 벗어난다.
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = PID_WDT_TIMEOUT_MS,
    .idle_core_mask = 0,
    .trigger_panic = true,
  };
  if (esp_task_wdt_reconfigure(&wdt_config) != ESP_OK &&
      esp_task_wdt_init(&wdt_config) != ESP_OK) {
    while (1) { Serial.println("[FAULT] TASK WDT INIT FAIL"); delay(1000); }
  }

  BaseType_t pidTaskOk = xTaskCreatePinnedToCore(pid_task, "PID", 4096, NULL, 2, NULL, 1);
  BaseType_t udpTaskOk = xTaskCreatePinnedToCore(udp_task, "UDP", 4096, NULL, 1, NULL, 0);
  if (pidTaskOk != pdPASS || udpTaskOk != pdPASS) {
    safety_lock = true;
    while (1) { Serial.println("[FAULT] TASK CREATE FAIL"); delay(1000); }
  }

  Serial.println("DUAL_IMU_CASCADE READY");
}

void loop() {
#if WIFI_LATENCY_DEBUG
  drainLatRing(udpLatRing);
  drainLatRing(pidLatRing);

  uint32_t nowMs = millis();
  if (nowMs - latLastSummaryMs >= 2000U) {
    latLastSummaryMs = nowMs;
    uint32_t dropped = __atomic_load_n(&udpLatRing.dropped, __ATOMIC_RELAXED)
                     + __atomic_load_n(&pidLatRing.dropped, __ATOMIC_RELAXED);
    Serial.printf("[LATSUM] up=%lu udpMaxUs=%lu pidMaxUs=%lu "
                  "udpLoopOver30=%lu udpLoopOver100=%lu udpLoopOver300=%lu "
                  "pidOver5=%lu dropped=%lu\n",
                  (unsigned long)nowMs,
                  (unsigned long)latUdpMaxUs, (unsigned long)latPidMaxUs,
                  (unsigned long)latUdpLoopOver30,
                  (unsigned long)latUdpLoopOver100,
                  (unsigned long)latUdpLoopOver300,
                  (unsigned long)latPidOver5, (unsigned long)dropped);
  }
  delay(100);
#else
  // UDP 객체는 udp_task 한 곳에서만 접근해 cross-core race를 피한다.
  delay(1000);
#endif
}
