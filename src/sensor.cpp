#include "../lib/sensor.h"

//==================================================================================
// ESP32, MPU-6500 - 실제 오차 적용 상보 필터 시뮬레이션
//==================================================================================

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>



//==================================================================================
//  updateSensor 함수 (시뮬레이션)
//==================================================================================
void updateSensor()
{
    dt = (micros() - loop_timer) / 1000000.0;
    loop_timer = micros();

    // --- 1. 실제 MPU-6500 오차 값을 시뮬레이션에 대입 ---

    // 가속도계 바이어스(오차) 시뮬레이션
    // 데이터시트상 일반적인 Zero-G Offset은 ±60mg (약 ±0.58 m/s²) 입니다.
    // X축과 Y축에 임의의 작은 오차값을 부여합니다.
    float accel_bias_x = 0.3;  // m/s^2
    float accel_bias_y = -0.2; // m/s^2

    // 자이로스코프 드리프트(오차) 시뮬레이션
    // 데이터시트상 일반적인 Zero-Rate Offset은 ±5 dps (약 ±0.087 rad/s) 입니다.
    // 각 축에 임의의 작은 각속도 오차를 부여합니다. 이것이 드리프트를 유발합니다.
    float gyro_drift_x = 0.03;   // rad/s (약 1.7 deg/s)
    float gyro_drift_y = -0.015; // rad/s (약 -0.8 deg/s)
    float gyro_drift_z = 0.02;   // rad/s (약 1.1 deg/s)

    // 센서 노이즈 시뮬레이션 (매번 미세하게 흔들리는 값)
    float accel_noise = ((rand() % 100) / 50.0 - 1.0) * 0.1; // -0.1 ~ +0.1 범위
    float gyro_noise = ((rand() % 100) / 50.0 - 1.0) * 0.01; // -0.01 ~ +0.01 범위

    // --- 2. 최종 시뮬레이션 값 생성 ---
    // (상황: 드론은 수평으로 정지해 있음)
    float simulated_accel_x = 0.0 + accel_bias_x + accel_noise;
    float simulated_accel_y = 0.0 + accel_bias_y + accel_noise;
    float simulated_accel_z = 9.81 + accel_noise; // Z축은 중력 + 노이즈

    float simulated_gyro_x = 0.0 + gyro_drift_x + gyro_noise;
    float simulated_gyro_y = 0.0 + gyro_drift_y + gyro_noise;
    float simulated_gyro_z = 0.0 + gyro_drift_z + gyro_noise;

    // --- 3. 가속도계 기반 각도 계산 ---
    float accel_angle_roll = atan2(simulated_accel_y, simulated_accel_z) * RAD_TO_DEG;
    float accel_angle_pitch = atan2(-simulated_accel_x, sqrt(simulated_accel_y * simulated_accel_y + simulated_accel_z * simulated_accel_z)) * RAD_TO_DEG;

    // --- 4. 자이로스코프 데이터 처리 (적분) ---
    angle_roll += simulated_gyro_x * dt;
    angle_pitch += simulated_gyro_y * dt;
    angle_yaw += simulated_gyro_z * dt;

    // --- 5. 상보 필터 적용 (센서 퓨전) ---
    angle_roll = COMPLEMENTARY_FILTER_ALPHA * angle_roll + (1.0 - COMPLEMENTARY_FILTER_ALPHA) * accel_angle_roll;
    angle_pitch = COMPLEMENTARY_FILTER_ALPHA * angle_pitch + (1.0 - COMPLEMENTARY_FILTER_ALPHA) * accel_angle_pitch;
}