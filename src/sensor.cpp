#include "../lib/sensor.h"

// MPU-6500 상보 필터 시뮬레이션
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// updateSensor 함수 (시뮬레이션)
void updateSensor()
{
    dt = (micros() - loop_timer) / 1000000.0;
    loop_timer = micros();

    // 1. MPU-6500 오차 시뮬레이션
    // 가속도계 바이어스(고정 오차)
    float accel_bias_x = 0.3;
    float accel_bias_y = -0.2;

    // 자이로스코프 드리프트(점진적 오차)
    float gyro_drift_x = 0.03;
    float gyro_drift_y = -0.015;
    float gyro_drift_z = 0.02;

    // 센서 노이즈(불규칙 오차)
    float accel_noise = ((rand() % 100) / 50.0 - 1.0) * 0.1;
    float gyro_noise = ((rand() % 100) / 50.0 - 1.0) * 0.01;

    // 2. 최종 센서 값 생성 (가정: 드론은 수평 정지 상태)
    float simulated_accel_x = 0.0 + accel_bias_x + accel_noise;
    float simulated_accel_y = 0.0 + accel_bias_y + accel_noise;
    float simulated_accel_z = 9.81 + accel_noise; // Z축 = 중력 + 노이즈

    float simulated_gyro_x = 0.0 + gyro_drift_x + gyro_noise;
    float simulated_gyro_y = 0.0 + gyro_drift_y + gyro_noise;
    float simulated_gyro_z = 0.0 + gyro_drift_z + gyro_noise;

    // 3. 가속도계 값으로 각도 계산
    float accel_angle_roll = atan2(simulated_accel_y, simulated_accel_z) * RAD_TO_DEG;
    float accel_angle_pitch = atan2(-simulated_accel_x, sqrt(simulated_accel_y * simulated_accel_y + simulated_accel_z * simulated_accel_z)) * RAD_TO_DEG;

    // 4. 자이로스코프 값 적분
    angle_roll += simulated_gyro_x * dt;
    angle_pitch += simulated_gyro_y * dt;
    angle_yaw += simulated_gyro_z * dt;

    // 5. 상보 필터로 센서 값 융합
    angle_roll = COMPLEMENTARY_FILTER_ALPHA * angle_roll + (1.0 - COMPLEMENTARY_FILTER_ALPHA) * accel_angle_roll;
    angle_pitch = COMPLEMENTARY_FILTER_ALPHA * angle_pitch + (1.0 - COMPLEMENTARY_FILTER_ALPHA) * accel_angle_pitch;
}