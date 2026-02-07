#include <Arduino.h>
#include <DShotRMT.h>

// 모터 4개 핀 정의 (4, 5, 6, 7)
DShotRMT motor1(GPIO_NUM_4, DSHOT600);
DShotRMT motor2(GPIO_NUM_5, DSHOT600);
DShotRMT motor3(GPIO_NUM_6, DSHOT600);
DShotRMT motor4(GPIO_NUM_7, DSHOT600);

void setup() {
  Serial.begin(115200);

  // 1. 초기화
  motor1.begin();
  motor2.begin();
  motor3.begin();
  motor4.begin();

  Serial.println("All Motors initialized.");

  // 2. 아밍 (기존 성공 코드 유지)
  Serial.println("Sending 0% throttle for 2 seconds to arm ESC...");

  motor1.sendThrottlePercent(0);
  motor2.sendThrottlePercent(0);
  motor3.sendThrottlePercent(0);
  motor4.sendThrottlePercent(0);

  delay(2000); // ESC가 Arming 될 때까지 2초 대기

  Serial.println("✅ ARMED!");
  Serial.println("⚠️ WARNING: ALL MOTORS 100% POWER STARTING IN 1 SECOND...");
  delay(1000); // 쫄리니까 1초만 마음의 준비
}

void loop() {
  // ==========================================
  // [무한반복] 4개 전부 100% 풀파워 전송
  // ==========================================
  
// loop 안의 코드를 이걸로 바꾸세요
motor1.sendThrottle(2000); // 2047이 MAX지만 안전하게 2000
motor2.sendThrottle(2000);
motor3.sendThrottle(2000);
motor4.sendThrottle(2000);
delayMicroseconds(200);
}