#include <Arduino.h>
#include <DShotRMT.h>
#include <Wire.h>
#include "MPU6500_WE.h"
#include <WiFi.h>
#include <esp_bt.h>

// ==========================================================
//  설정
// ==========================================================
const gpio_num_t MOTOR_PIN = GPIO_NUM_4;
DShotRMT motor(MOTOR_PIN, DSHOT600);
MPU6500_WE mpu = MPU6500_WE(0x68);

// ==========================================================
//  상태 알림 함수 (모터 까딱이기)
// ==========================================================
void signal_alive(int times) {
  for(int i=0; i<times; i++) {
    // 0.1초 동안 살짝 돌리기 (소리와 움직임 발생)
    motor.sendThrottlePercent(10); 
    delay(100);
    motor.sendThrottlePercent(0);
    delay(200);
  }
  delay(500); // 구분을 위한 대기
}

// ==========================================================
//  Core 0 Task (센서 & 계산)
// ==========================================================
volatile int global_throttle = 0; // 공유 변수

void pid_task(void *pvParameters) {
  for (;;) {
    // 1. 센서 읽기 (실패해도 멈추지 않음)
    xyzFloat acc = mpu.getGValues();
    xyzFloat gyro = mpu.getGyrValues();

    // 2. 간단한 각도 계산
    float angle_pitch = atan2(-acc.x, sqrt(acc.y * acc.y + acc.z * acc.z)) * RAD_TO_DEG;
    
    // 3. 테스트용: 기울기에 따라 스로틀 변경 (P제어만)
    int throttle = 15; // 기본 스로틀
    if (abs(angle_pitch) > 30) throttle = 0; // 30도 넘으면 정지 (안전)
    
    global_throttle = throttle;
    vTaskDelay(1);
  }
}

// ==========================================================
//  Setup (Core 1)
// ==========================================================
void setup() {
  // 1. 전원 안정화 대기 (매우 중요: 배터리 연결 직후 전압 흔들림 방지)
  WiFi.mode(WIFI_OFF);
  btStop();
  delay(3000); 

  // 2. 모터 초기화
  motor.begin();
  
  // [생존신고 1] 모터 초기화 됨 -> "윙" (1번)
  signal_alive(1);

  // 3. MPU6500 초기화
  Wire.begin(8, 9);
  
  // ★★★ 중요: MPU 연결 안 돼도 멈추지 않고 넘어감! ★★★
  if (!mpu.init()) {
    // 실패 시: 모터가 미친듯이 3번 뜀
    signal_alive(3); 
    // 여기서 멈추지 않고 그냥 진행시킴 (비행은 안되겠지만 모터는 돌아야 함)
  } else {
    // 성공 시: 모터가 점잖게 2번 뜀
    signal_alive(2); 
    mpu.autoOffsets();
  }

  // 4. Arming (준비)
  for(int i=0; i<20; i++) {
    motor.sendThrottlePercent(0);
    delay(50);
  }
  
  // [생존신고 3] Arming 완료 -> "위이잉" (길게)
  motor.sendThrottlePercent(15);
  delay(500);
  motor.sendThrottlePercent(0);
  delay(500);

  // 5. PID 태스크 시작
  xTaskCreatePinnedToCore(pid_task, "PID", 4096, NULL, 1, NULL, 0);
}

// ==========================================================
//  Loop (Core 1)
// ==========================================================
void loop() {
  // 공유 변수 읽어서 모터로 전송
  int th = global_throttle;
  
  // 최소한의 스로틀 보장 (테스트용)
  if (th < 5) th = 5; 

  motor.sendThrottlePercent(th);
  delayMicroseconds(100);
}