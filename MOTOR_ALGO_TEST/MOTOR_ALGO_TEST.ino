#include <Arduino.h>
#include <DShotRMT.h>
#include "../lib/motor.h"

// === 핀 설정 ===
const int MOTOR_PIN = 4; 

// === DShot 설정 ===
// DShotRMT 라이브러리를 사용하여 모터 객체 생성
DShotRMT dshot_motor(MOTOR_PIN, DSHOT600);

// === 알고리즘 계산기 설정 ===
// motor.cpp의 알고리즘을 사용하기 위한 객체. 이름 충돌 방지를 위해 이름 변경.
Motor motor_calculator;
const float VMAX = 12.6f; // motor.cpp에 정의된 최대 전압값

// === 테스트용 목표 움직임 값 ===
float target_vx = 0.0f; // 좌-우 이동
float target_vy = 0.0f; // 앞-뒤 이동
float target_vz = 0.8f; // 상승-하강 (초기 시동을 위해 약간의 추력)
float target_wx = 0.0f; // Roll (좌우 기울기)
float target_wy = 0.0f; // Pitch (앞뒤 기울기)
float target_wz = 0.0f; // Yaw (제자리 회전)

void setup() {
  Serial.begin(115200);
  Serial.println("Motor Algorithm Test (DShot on GPIO 4)");

  // --- 1. DShot 모터 초기화 ---
  dshot_motor.begin();
  Serial.printf("DShot motor initialized on GPIO %d\n", MOTOR_PIN);

  // --- 2. ESC Arming 절차 ---
  // DShot 프로토콜을 사용하여 0% 스로틀을 2초간 보내 ESC를 Arming합니다.
  Serial.println("Arming ESC with 0% throttle for 2 seconds...");
  dshot_motor.sendThrottlePercent(0);
  delay(2000);
  Serial.println("Arming complete.");
}

void loop() {
  // --- 1. motor.cpp의 핵심 알고리즘 호출 ---
  // 목표 움직임 값을 기반으로 4개 모터의 "가상" 전압을 계산.
  // motor.cpp의 ledcWrite는 주석 처리되어 실제 신호는 보내지 않음.
  motor_calculator.setVoltage(target_vx, target_vy, target_vz, target_wx, target_wy, target_wz);

  // --- 2. 계산된 전압을 DShot 스로틀로 변환 ---
  // 알고리즘이 계산한 4개의 모터 전압 중 첫 번째(M1) 값만 사용.
  float calculated_voltage = motor_calculator.voltage[0];
  
  // 전압(0~12.6V)을 스로틀(0~100%)로 변환
  int throttle_percent = (calculated_voltage / VMAX) * 100;
  
  // 스로틀 값은 0~100 사이로 제한
  throttle_percent = constrain(throttle_percent, 0, 100);

  // --- 3. DShot 명령 전송 ---
  dshot_motor.sendThrottlePercent(throttle_percent);

  // --- 4. 알고리즘 결과 및 DShot 출력 값 확인 ---
  Serial.printf("Target Vz: %.2f -> M1 Voltage: %.2fV -> DShot Throttle: %d%%\n", 
                target_vz, calculated_voltage, throttle_percent);

  // 100ms 마다 루프 실행 (10Hz)
  delay(100);
}