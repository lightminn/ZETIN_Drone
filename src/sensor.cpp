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

//--- 자세 각도 변수 ---
float angle_roll = 0;
float angle_pitch = 0;
float angle_yaw = 0; // Yaw는 가속도계로 보정 불가하여 드리프트 발생

//--- 시간 제어 변수 ---
unsigned long loop_timer;
float dt; // 루프 실행 시간 (초)

//==================================================================================
//  SETUP 함수
//==================================================================================
void setup(void)
{
    Serial.begin(115200);
    while (!Serial)
        ;

    if (!mpu.begin())
    {
        Serial.println("MPU6500을 찾지 못했습니다. 연결을 확인하세요.");
        while (1)
        {
            delay(10);
        }
    }
    Serial.println("MPU6500이 성공적으로 초기화되었습니다.");

    // 센서 범위 설정
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG); // 이전 대화에서 확인된 상수 사용
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

    delay(100);

    // 첫 루프의 dt 계산을 위한 타이머 초기화
    loop_timer = micros();

    // CSV 출력 헤더
    Serial.println("Roll,Pitch,Yaw");
}

//==================================================================================
//  LOOP 함수
//==================================================================================
void loop()
{
    // --- 1. 시간차 계산 (dt) ---
    // 이전 루프와 현재 루프 사이의 시간차를 초 단위로 계산
    dt = (micros() - loop_timer) / 1000000.0;
    loop_timer = micros();

    // --- 2. 센서 데이터 읽기 ---
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // --- 3. 가속도계 기반 각도 계산 ---
    // 가속도 데이터로부터 Roll과 Pitch 각도를 계산합니다.
    // 이 값은 노이즈가 심하지만, 중력을 기준으로 하므로 드리프트가 없습니다.
    float accel_angle_roll = atan2(a.acceleration.y, a.acceleration.z) * RAD_TO_DEG;
    float accel_angle_pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * RAD_TO_DEG;

    // --- 4. 자이로스코프 데이터 처리 ---
    // 자이로 데이터를 적분하여 각도 변화량을 계산합니다.
    // 이 값은 단기적으로 매우 정확하지만, 시간이 지남에 따라 드리프트가 누적됩니다.
    angle_roll += g.gyro.x * dt;
    angle_pitch += g.gyro.y * dt;
    angle_yaw += g.gyro.z * dt; // Yaw는 보정할 방법이 없어 계속 드리프트됨

    // --- 5. 상보 필터 적용 (센서 퓨전) ---
    // 자이로의 누적 오차를 가속도계 값으로 계속해서 보정해줍니다.
    // 98%는 자이로의 빠른 반응을 믿고, 2%는 가속도계의 절대 각도로 보정합니다.
    angle_roll = 0.98 * angle_roll + 0.02 * accel_angle_roll;
    angle_pitch = 0.98 * angle_pitch + 0.02 * accel_angle_pitch;

    // --- 6. 결과 출력 ---
    Serial.print(angle_roll);
    Serial.print(",");
    Serial.print(angle_pitch);
    Serial.print(",");
    Serial.println(angle_yaw);

    // 루프 주기를 일정하게 유지 (선택 사항)
    // delay(4);
}