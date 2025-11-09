/*
  ZETIN Drone - Standalone Motor Algorithm Test (Dual Core)
*/

#include <Arduino.h>
#include <DShotRMT.h> 
#include <cmath>     
#include <WiFi.h>      // 듀얼 코어 사용 시 전원 안정화를 위해 추가
#include <esp_bt.h>

// =========================================================================
//  MOTOR CONTROL ALGORITHM (수정 없음)
// =========================================================================

namespace
{
    // --- Algorithm Constants ---
    constexpr float vMax = 12.6f;
    constexpr float dVmax = 0.05f; 

    /* S : 6×4 */
    constexpr float S[6][4] = {
        {0.12f, -0.12f, -0.12f, 0.12f},     // v_x
        {-0.12f, 0.12f, -0.12f, 0.12f},     // v_y
        {0.25f, 0.25f, 0.25f, 0.25f},       // v_z
        {-0.015f, 0.015f, 0.015f, -0.015f}, // w_x
        {0.015f, -0.015f, 0.015f, -0.015f}, // w_y
        {-0.02f, 0.02f, -0.02f, 0.02f}      // w_z
    };
    /* W, R */
    constexpr float W[6] = {1, 1, 1, 0.3f, 0.3f, 0.3f};
    constexpr float R[4] = {0.01f, 0.01f, 0.01f, 0.01f};

    template <typename T>
    constexpr const T &custom_clamp(const T &v, const T &lo, const T &hi)
    {
        return (v < lo) ? lo : (hi < v) ? hi : v;
    }
}

// --- Motor Class Definition (수정 없음) ---
class Motor
{
public:
    Motor();
    ~Motor();
    void setVoltage(float v_x, float v_y, float v_z,
                    float w_x, float w_y, float w_z);
    void start();
    void stop();
    float voltage[4];

private:
    int motor_num;
};

// --- Motor Class Implementation (수정 없음) ---
Motor::Motor() : voltage{0, 0, 0, 0}, motor_num(0) {}
Motor::~Motor() {}

void Motor::setVoltage(float v_x, float v_y, float v_z,
                       float w_x, float w_y, float w_z)
{
    const float x[6] = {v_x, v_y, v_z, w_x, w_y, w_z};
    float H[4][4] = {0};
    float f[4] = {0};

    for (int i = 0; i < 4; ++i)
    {
        for (int j = i; j < 4; ++j)
        {
            float hij = (i == j ? R[i] : 0.f);
            for (int k = 0; k < 6; ++k)
                hij += S[k][i] * W[k] * S[k][j];
            H[i][j] = H[j][i] = hij;
        }
        float sum = 0.f;
        for (int k = 0; k < 6; ++k)
            sum += S[k][i] * W[k] * x[k];
        f[i] = sum;
    }

    float L[4][4] = {0};
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j <= i; ++j)
        {
            float s = H[i][j];
            for (int k = 0; k < j; ++k)
                s -= L[i][k] * L[j][k];
            L[i][j] = (i == j) ? std::sqrt(s) : s / L[j][j];
        }
    }
    float y[4];
    for (int i = 0; i < 4; ++i)
    {
        float s = f[i];
        for (int k = 0; k < i; ++k)
            s -= L[i][k] * y[k];
        y[i] = s / L[i][i];
    }
    
    float V_optimal[4]; 
    for (int i = 3; i >= 0; --i)
    {
        float s = y[i];
        for (int k = i + 1; k < 4; ++k)
            s -= L[k][i] * V_optimal[k]; 
        
        V_optimal[i] = s / L[i][i]; 
    }

    for (int m = 0; m < 4; ++m)
    {
        voltage[m] = custom_clamp(V_optimal[m], 0.f, vMax);
    }
}

void Motor::start() {}
void Motor::stop() {}


// =========================================================================
//  MAIN SKETCH (듀얼 코어 구조로 수정됨)
// =========================================================================

const int MOTOR_PIN = 4; 
DShotRMT dshot_motor(MOTOR_PIN, DSHOT600);
Motor motor_calculator;

// === Test Target Values ===
float target_vx = 0.0f; 
float target_vy = 0.0f; 
float target_vz = 0.0f; 
float target_wx = 0.0f; 
float target_wy = 0.0f; 
float target_wz = 0.0f; 

unsigned long lastPrintTime = 0;
const long printInterval = 100; 

// --- ★★★ 듀얼 코어용 공유 변수 ★★★ ---
// Core 0 (계산) -> Core 1 (모터 전송)
volatile int global_throttle_percent = 0; 
// Core 1 (모터 준비) -> Core 0 (시작 대기)
volatile bool dshot_task_ready = false; 

// ==========================================================
//  Core 1: DShot 모터 제어 전용 태스크
// ==========================================================
void dshot_task(void *pvParameters) {
  Serial.println("[Core 1] DShot Task starting.");
  
  // 1. 모터 초기화 (Core 1에서)
  dshot_motor.begin();
  Serial.printf("[Core 1] DShot motor initialized on GPIO %d\n", MOTOR_PIN);

  // 2. Arming (Core 1에서)
  Serial.println("[Core 1] Arming ESC (sending 0% for 2s)...");
  dshot_motor.sendThrottlePercent(0);
  delay(2000);
  Serial.println("[Core 1] Arming complete. Signaling Core 0.");
  
  // 3. Core 0에게 준비 완료 신호
  dshot_task_ready = true;

  // 4. Core 1 무한 루프
  for (;;) {
    // Core 0이 계산한 최신 스로틀 값을 읽어와서 전송
    dshot_motor.sendThrottlePercent(global_throttle_percent);
    
    // DShot 신호를 250Hz (4ms) 주기로 안정적으로 전송
    // (ESC Failsafe 방지)
    delayMicroseconds(200); 
  }
}


// ==========================================================
//  Core 0: SETUP 함수
// ==========================================================
void setup() {
  Serial.begin(115200);

  // --- 0. 전원 안정화 ---
  WiFi.mode(WIFI_OFF);
  btStop(); 
  Serial.println("[Core 0] Wi-Fi & BT OFF.");
  Serial.println("Standalone Motor Algorithm Test (Dual Core)");

  // --- 1. DShot 태스크를 Core 1에 생성 ---
  xTaskCreatePinnedToCore(
      dshot_task,         // 실행할 함수
      "DShotTask",        // 태스크 이름
      4096,               // 스택 크기 (Guru Meditation Error 방지)
      NULL,               // 파라미터
      1,                  // 우선순위
      NULL,               // 핸들
      1);                 // Core ID (1)

  // --- 2. Core 1이 모터 Arming을 마칠 때까지 대기 ---
  Serial.println("[Core 0] Waiting for Core 1 (DShot) to arm...");
  while (dshot_task_ready == false) {
    delay(100);
  }
  Serial.println("[Core 0] Core 1 is ready. Starting main loop.");
  
  lastPrintTime = millis(); // 메인 루프의 프린트 타이머 초기화
}


// ==========================================================
//  Core 0: LOOP 함수 (계산 전용)
// ==========================================================
void loop() {
  // --- 0. 사인파(Sine Wave)로 target_vz 값 자동 증감 ---
  float time_in_seconds = millis() / 1000.0f;
  
  // (사용자가 마지막으로 수정한 값 적용)
  target_vz = 5.8f + (sin(time_in_seconds * 1.0f) * 5.f);

  // 1. Call the core algorithm (Core 0에서 계산)
  motor_calculator.setVoltage(target_vx, target_vy, target_vz, target_wx, target_wy, target_wz);

  // 2. Convert the calculated voltage to a DShot throttle percentage
  float calculated_voltage = motor_calculator.voltage[0];
  int throttle_percent = (calculated_voltage / vMax) * 100;
  throttle_percent = constrain(throttle_percent, 0, 100);

  // 3. Send the command via DShot (!!! 제거 !!!)
  // dshot_motor.sendThrottlePercent(throttle_percent); 
  
  // --- ★★★ 3. 계산된 값을 Core 1이 사용할 수 있도록 공유 변수에 업데이트 ★★★ ---
  global_throttle_percent = throttle_percent;

  // 4. Print results to Serial Monitor periodically
  if (millis() - lastPrintTime >= printInterval) {
    lastPrintTime = millis();
    Serial.printf("Target Vz: %.2f -> M1 Voltage: %.2fV -> DShot Throttle: %d%%\n", 
                  target_vz, calculated_voltage, throttle_percent);
  }

  // 5. Run control loop delay (사용자가 마지막으로 수정한 값 적용)
  delayMicroseconds(200);
}