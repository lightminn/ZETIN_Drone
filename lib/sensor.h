#ifndef SENSOR_H
#define SENSOR_H

// 필요한 라이브러리들을 헤더 파일에 포함시킵니다.
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// --- 전역 객체 및 변수 선언 (extern) ---
// 다른 파일에서 이 변수들을 사용할 수 있도록 '선언'만 합니다.
// 실제 변수는 메인 .cpp 파일에 있습니다.
extern Adafruit_MPU6050 mpu;
extern float angle_roll;
extern float angle_pitch;
extern float angle_yaw;
extern unsigned long loop_timer;
extern float dt;

// --- 전역 함수 원형 선언 ---
// 메인 파일에 있는 setup()과 loop() 함수를 선언합니다.
void updateSensor();

#endif // SENSOR_H