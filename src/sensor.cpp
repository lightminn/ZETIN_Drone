//==================================================================================
// ESP32, MPU-6500 및 상보 필터 예제 코드
//==================================================================================
// 설명: MPU-6500의 가속도계와 자이로스코프 데이터를 상보 필터로 융합하여,
//      드리프트가 보정된 안정적인 Roll/Pitch 각도를 계산하고 출력합니다.
//==================================================================================

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// MPU6050 객체 생성
Adafruit_MPU6050 mpu;

// 상보 필터 계수 (조정 가능)
float COMPLEMENTARY_FILTER_ALPHA = 0.98;

//--- 자세 각도 변수 ---
float angle_roll = 0;
float angle_pitch = 0;
float angle_yaw = 0; // Yaw는 가속도계로 보정 불가하여 드리프트 발생

//--- 시간 제어 변수 ---
unsigned long loop_timer;
float dt; // 루프 실행 시간 (초)



//==================================================================================
//  updateSensor 함수
//==================================================================================
void updateSensor()
{
    // --- 1. 시간차 계산 (dt) ---
    // 이전 루프와 현재 루프 사이의 시간차를 초 단위로 계산
    dt = (micros() - loop_timer) / 1000000.0;
    loop_timer = micros();

    // --- 2. 센서 데이터 읽기 ---
    // 센서 데이터 시뮬레이션 (물리 센서 없이 테스트용)
    // 실제 센서 데이터 대신 가상의 값을 사용합니다.
    // 이 값들을 변경하여 다양한 상황을 시뮬레이션할 수 있습니다.
    float simulated_accel_x = 0.0; // 중력 방향에 따라 조정
    float simulated_accel_y = 0.0;
    float simulated_accel_z = 9.81; // 지구 중력 가속도

    float simulated_gyro_x = 0.0; // 각속도 (rad/s)
    float simulated_gyro_y = 0.0;
    float simulated_gyro_z = 0.0;

    // 시뮬레이션된 데이터를 사용하여 가속도계 기반 각도 계산
    float a_acceleration_x = simulated_accel_x;
    float a_acceleration_y = simulated_accel_y;
    float a_acceleration_z = simulated_accel_z;

    // 시뮬레이션된 데이터를 사용하여 자이로스코프 데이터 처리
    float g_gyro_x = simulated_gyro_x;
    float g_gyro_y = simulated_gyro_y;
    float g_gyro_z = simulated_gyro_z;

    // --- 3. 가속도계 기반 각도 계산 ---
    // 가속도 데이터로부터 Roll과 Pitch 각도를 계산합니다.
    // 이 값은 노이즈가 심하지만, 중력을 기준으로 하므로 드리프트가 없습니다.
    float accel_angle_roll = atan2(a_acceleration_y, a_acceleration_z) * RAD_TO_DEG;
    float accel_angle_pitch = atan2(-a_acceleration_x, sqrt(a_acceleration_y * a_acceleration_y + a_acceleration_z * a_acceleration_z)) * RAD_TO_DEG;

    // --- 4. 자이로스코프 데이터 처리 ---
    // 자이로 데이터를 적분하여 각도 변화량을 계산합니다.
    // 이 값은 단기적으로 매우 정확하지만, 시간이 지남에 따라 드리프트가 누적됩니다.
    angle_roll += g_gyro_x * dt;
    angle_pitch += g_gyro_y * dt;
    angle_yaw += g_gyro_z * dt; // Yaw는 보정할 방법이 없어 계속 드리프트됨

    // --- 5. 상보 필터 적용 (센서 퓨전) ---
    // 자이로의 누적 오차를 가속도계 값으로 계속해서 보정해줍니다.
    // 98%는 자이로의 빠른 반응을 믿고, 2%는 가속도계의 절대 각도로 보정합니다.
    angle_roll = COMPLEMENTARY_FILTER_ALPHA * angle_roll + (1.0 - COMPLEMENTARY_FILTER_ALPHA) * accel_angle_roll;
    angle_pitch = COMPLEMENTARY_FILTER_ALPHA * angle_pitch + (1.0 - COMPLEMENTARY_FILTER_ALPHA) * accel_angle_pitch;

    // --- 6. 결과 출력 ---
    Serial.print(angle_roll);
    Serial.print(",");
    Serial.print(angle_pitch);
    Serial.print(",");
    Serial.println(angle_yaw);

    // 루프 주기를 일정하게 유지 (선택 사항)
    // delay(4);
}