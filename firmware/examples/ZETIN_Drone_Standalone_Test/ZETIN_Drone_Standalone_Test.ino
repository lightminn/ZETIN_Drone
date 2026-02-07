/*
  ZETIN Drone - Standalone Motor Algorithm Test (Dual Core Fixed)
  - Core 0: Math & Algorithm (1kHz)
  - Core 1: DShot Signal (High Speed, ~10kHz with 75us delay)
*/

#include <Arduino.h>
#include <DShotRMT.h> 
#include <cmath>     
#include <WiFi.h>
#include <esp_bt.h>

// =========================================================================
//  MOTOR CONTROL ALGORITHM (기존과 동일)
// =========================================================================
namespace
{
    constexpr float vMax = 12.6f;
    // ... (S, W, R 배열 등 기존 상수 생략, 그대로 사용) ...
    constexpr float S[6][4] = {
        {0.12f, -0.12f, -0.12f, 0.12f}, {-0.12f, 0.12f, -0.12f, 0.12f},
        {0.25f, 0.25f, 0.25f, 0.25f}, {-0.015f, 0.015f, 0.015f, -0.015f},
        {0.015f, -0.015f, 0.015f, -0.015f}, {-0.02f, 0.02f, -0.02f, 0.02f}
    };
    constexpr float W[6] = {1, 1, 1, 0.3f, 0.3f, 0.3f};
    constexpr float R[4] = {0.01f, 0.01f, 0.01f, 0.01f};

    template <typename T>
    constexpr const T &custom_clamp(const T &v, const T &lo, const T &hi) {
        return (v < lo) ? lo : (hi < v) ? hi : v;
    }
}

class Motor {
public:
    Motor() : voltage{0} {}
    float voltage[4];
    void setVoltage(float v_x, float v_y, float v_z, float w_x, float w_y, float w_z); // 구현은 아래
};

// (setVoltage 구현부는 너무 기므로 기존 코드를 그대로 사용한다고 가정)
// 실제 컴파일 시에는 기존 코드의 setVoltage 구현 내용이 여기에 있어야 합니다.
void Motor::setVoltage(float v_x, float v_y, float v_z, float w_x, float w_y, float w_z) {
    // ... (기존의 복잡한 행렬 연산 코드 복사) ...
    // 테스트를 위해 간단히 전압을 v_z로 설정하는 가상 코드 (실제론 원래 코드를 쓰세요)
    const float x[6] = {v_x, v_y, v_z, w_x, w_y, w_z};
    // ... 행렬 연산 후 ...
    // 여기서는 테스트용으로 v_z가 바로 반영되도록 가정
    float calculated = v_z; 
    for(int i=0; i<4; i++) voltage[i] = custom_clamp(calculated, 0.f, vMax);
}


// =========================================================================
//  DUAL CORE SETUP
// =========================================================================

const int MOTOR_PIN = 4; 
DShotRMT dshot_motor(MOTOR_PIN, DSHOT600);

// [공유 변수] Core 0에서 쓰고 -> Core 1에서 읽음
// volatile: 컴파일러 최적화 방지 (즉시 메모리 읽기)
volatile int global_throttle_percent = 0; 
volatile float debug_vz = 0.0f;

// ==========================================================
//  Core 0 Task: 수학 계산 (Math Task)
// ==========================================================
void math_task(void *pvParameters) {
  Motor local_calculator; // 이 태스크 전용 계산기 인스턴스
  
  Serial.println("[Core 0] Math Task running...");

  // 정확한 주기 실행을 위한 변수 (1kHz = 1ms 주기)
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1); 

  for (;;) {
    // 1. 사인파 생성 (시간 기반)
    float time_in_seconds = millis() / 1000.0f;
    float target_vz = 5.8f + (sin(time_in_seconds * 1.0f) * 5.0f);
    
    // 2. 알고리즘 계산 수행
    local_calculator.setVoltage(0, 0, target_vz, 0, 0, 0);
    
    // 3. 결과 변환
    float calc_voltage = local_calculator.voltage[0];
    int throttle = (calc_voltage / vMax) * 100;
    throttle = constrain(throttle, 0, 100);

    // 4. 공유 변수 업데이트 (Atomic에 가깝게 단순 대입)
    global_throttle_percent = throttle;
    debug_vz = target_vz; // 디버깅용

    // 5. OS에게 제어권 반환 (중요: 여기서 대기해야 Core 0의 다른 시스템 작업도 돔)
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ==========================================================
//  Core 1: Setup & Loop (DShot 전송 담당)
// ==========================================================
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_OFF);
  btStop();

  Serial.println("=== Dual Core Fast DShot Test ===");

  // 1. 모터 초기화 (Core 1)
  dshot_motor.begin();
  
  // 2. Arming (안전을 위해 0 전송)
  Serial.println("Arming ESC...");
  for(int i=0; i<100; i++) {
    dshot_motor.sendThrottlePercent(0);
    delay(20); 
  }
  Serial.println("Arming Done. Starting Math Task.");

  // 3. Core 0에 계산 태스크 생성
  // 중요: Loop와 싸우지 않게 아예 다른 코어(0)로 보냄
  xTaskCreatePinnedToCore(
      math_task,    // 실행할 함수
      "MathTask",   // 이름
      10000,        // 스택 크기
      NULL,         // 파라미터
      1,            // 우선순위
      NULL,         // 핸들
      0);           // ★ Core 0에 할당 ★
}

void loop() {
  // ============================================================
  //  Core 1: DShot High Speed Loop
  // ============================================================
  
  // 1. Core 0이 계산해둔 최신 스로틀 값 읽기
  int throttle = global_throttle_percent;

  // 2. DShot 전송 (Non-blocking에 가까움, RMT 사용)
  dshot_motor.sendThrottlePercent(throttle);

  // 3. 사용자 요청: 짧은 딜레이 (약 10~13kHz 루프)
  // loop() 함수는 끝나면 다시 호출되면서 약간의 틈이 생겨 WDT 리셋을 방지해줍니다.
  // 하지만 안전을 위해 최소한의 마이크로초 대기는 필요합니다.
  delayMicroseconds(75); 

  // 4. 디버깅 (너무 자주 출력하면 모터 제어 끊기므로 가끔만)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    // Serial.printf는 다소 느리니 실제 비행시는 주석 처리 권장
    Serial.printf("Tgt Vz: %.2f -> Thr: %d%%\n", debug_vz, throttle);
  }
}