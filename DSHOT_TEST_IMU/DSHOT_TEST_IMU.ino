#include <Arduino.h>
#include <DShotRMT.h>
#include <Wire.h>
#include "MPU6500_WE.h" 
#include <WiFi.h>      // (전원 안정화를 위해 여전히 끄는 것을 권장)
#include <esp_bt.h>

// --- 모터 설정 ---
const gpio_num_t MOTOR_PIN = GPIO_NUM_4;
DShotRMT motor(MOTOR_PIN, DSHOT600);

// --- MPU6500 설정 ---
MPU6500_WE mpu = MPU6500_WE(0x68); 

// --- 각도 계산 변수 ---
float angle_pitch = 0.0;
float angle_roll = 0.0;
float angle_pitch_acc = 0.0;
float angle_roll_acc = 0.0;
unsigned long prev_time = 0; 
const float ALPHA = 0.98;

// --- 모터 제어 변수 ---
const char CONTROL_AXIS = 'Y'; 
const int MIN_THROTTLE_PERCENT = 5;  
const int MAX_THROTTLE_PERCENT = 40; 
const float MAX_ANGLE = 45.0; 

// --- ★★★ 듀얼 코어용 공유 변수 ★★★ ---
// 'volatile' 키워드는 두 코어가 데이터를 안전하게 공유하게 함
volatile int global_throttle_percent = 0; 

// ==========================================================
//  Core 1: DShot 모터 제어 전용 태스크
// ==========================================================
void dshot_task(void *pvParameters) {
  Serial.println("DShot Task started on Core 1.");
  
  // 이 태스크는 Core 1에서 무한 반복
  for (;;) {
    // Core 0에서 계산한 최신 스로틀 값을 읽어서 모터로 전송
    // motor.sendThrottlePercent(global_throttle_percent);
        motor.sendThrottlePercent(1);
    
    // DShot 신호를 약 250Hz (4ms) 주기로 안정적으로 전송
    // (ESC의 Failsafe 타임아웃 방지)
    delay(4); 
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
  Serial.println("Wi-Fi & BT OFF. Power stabilized.");

  // --- 1. MPU6500 초기화 (Core 0) ---
  Wire.begin(8, 9); 
  if (!mpu.init()) { 
    Serial.println("Failed to find MPU6500 chip");
    while (1) { delay(10); }
  }
  Serial.println("MPU6500 Found!");

  mpu.autoOffsets(); 
  Serial.println("Calibration done!");

  mpu.setAccRange(MPU6500_ACC_RANGE_8G);    
  mpu.setGyrRange(MPU6500_GYRO_RANGE_500); 
  mpu.enableGyrDLPF(); 
  mpu.setGyrDLPF(MPU6500_DLPF_2);     
  
  // --- 2. 모터 초기화 및 Arming (Core 0) ---
  motor.begin();
  Serial.println("Motor initialized.");
  Serial.println("Sending 0% throttle for 2 seconds to arm ESC...");
  motor.sendThrottlePercent(0); 
  delay(2000); 
  Serial.println("Arming complete! Starting DShot Task on Core 1...");

  // --- 3. DShot 태스크를 Core 1에 생성 및 실행 ---
  xTaskCreatePinnedToCore(
      dshot_task,         // 실행할 함수
      "DShotTask",        // 태스크 이름 (디버깅용)
      1024,               // 스택 크기
      NULL,               // 파라미터 없음
      1,                  // 우선순위 1
      NULL,               // 태스크 핸들 (필요 없음)
      1);                 // 실행할 코어 ID (0 또는 1)

  prev_time = micros(); 
}

// ==========================================================
//  Core 0: LOOP 함수 (센서 계산 전용)
// ==========================================================
void loop() {
  // --- 1. 시간 계산 ---
  unsigned long current_time = micros();
  float dt = (current_time - prev_time) / 1000000.0; 
  prev_time = current_time;

  // --- 2. 센서 값 읽기 (I2C 통신, Core 0에서만) ---
  xyzFloat acc = mpu.getGValues();    
  xyzFloat gyro = mpu.getGyrValues(); 

  // --- 3. 각도 계산 (상보 필터, Core 0에서만) ---
  angle_roll_acc = atan2(acc.y, acc.z) * RAD_TO_DEG;
  angle_pitch_acc = atan2(-acc.x, sqrt(acc.y * acc.y + acc.z * acc.z)) * RAD_TO_DEG;
  
  angle_pitch = ALPHA * (angle_pitch + gyro.y * dt) + (1.0 - ALPHA) * angle_pitch_acc;
  angle_roll = ALPHA * (angle_roll + gyro.x * dt) + (1.0 - ALPHA) * angle_roll_acc;

  // --- 4. 각도를 스로틀로 변환 (Core 0에서만) ---
  float control_angle = 0.0;
  if (CONTROL_AXIS == 'Y') {
    control_angle = angle_pitch;
  } else {
    control_angle = angle_roll; 
  }

  float tilt_angle = abs(control_angle);
  
  int throttle_percent = map(tilt_angle, 0.0, MAX_ANGLE, MIN_THROTTLE_PERCENT, MAX_THROTTLE_PERCENT);
  throttle_percent = constrain(throttle_percent, MIN_THROTTLE_PERCENT, MAX_THROTTLE_PERCENT);
  
  if (tilt_angle < 2.0) {
    throttle_percent = MIN_THROTTLE_PERCENT; 
  }

  // --- 5. 모터 제어 (이제 안 함) ---
  // motor.sendThrottlePercent(throttle_percent); // <-- 이 코드가 충돌을 일으켰음!

  // --- 6. Core 1이 읽어갈 수 있도록 공유 변수만 업데이트 ---
  global_throttle_percent = throttle_percent;

  // --- 7. 상태 출력 (Core 0) ---
  Serial.print("Axis: "); Serial.print(CONTROL_AXIS);
  Serial.print(" | Angle: "); Serial.print(control_angle);
  Serial.print(" | Throttle: "); Serial.println(throttle_percent);
  
  // 이 딜레이는 Core 0에만 적용됨 (Core 1은 4ms로 계속 도는 중)
  delay(10); 
}