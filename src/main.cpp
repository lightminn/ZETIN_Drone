#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "../lib/sensor.h"
#include "../lib/connect.h"

Adafruit_MPU6050 mpu;

// 상보 필터 계수
float COMPLEMENTARY_FILTER_ALPHA = 0.98;

// 자세각 변수
float angle_roll = 0;
float angle_pitch = 0;
float angle_yaw = 0;

// 시간 제어 변수
unsigned long loop_timer;
float dt;

void setup()
{
  Serial.begin(115200);
  // MPU6500 초기화 건너뛰기 (센서 없이 테스트)
  Serial.println("Skipping MPU6500 initialization (sensorless test mode).");

  delay(100);

  // 루프 시간 간격(dt) 계산을 위한 타이머 초기화
  loop_timer = micros();

  // CSV 출력 헤더
  Serial.println("Roll,Pitch,Yaw");

  delay(1000);

  // WiFi 및 TCP 서버 설정
  setupWiFi();
  startTCPServer();
}

void loop()
{
  handleClientConnection();
  // updateSensor();

  // 결과 출력
  // Serial.print(angle_roll);
  // Serial.print(",");
  // Serial.print(angle_pitch);
  // Serial.print(",");
  // Serial.println(angle_yaw);

  delay(10);
}
