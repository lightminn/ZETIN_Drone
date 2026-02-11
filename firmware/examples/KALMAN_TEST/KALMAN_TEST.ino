#include <Arduino.h>
#include <SPI.h>
#include <ICM42670P.h>

// ==========================================================
// 1. 칼만 필터 클래스 (복붙)
// ==========================================================
class Kalman {
public:
    Kalman() {
        P[0][0] = 0.0f; P[0][1] = 0.0f;
        P[1][0] = 0.0f; P[1][1] = 0.0f;
        angle = 0.0f;
        bias = 0.0f;
    }

    // newAngle: 가속도계로 구한 각도 (노이즈 심함)
    // newRate: 자이로스코프로 구한 각속도 (반응 빠름)
    // dt: 루프 시간 간격 (초 단위)
    float getAngle(float newAngle, float newRate, float dt) {
        // 1. 예측 (Predict)
        rate = newRate - bias;
        angle += dt * rate;

        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;

        // 2. 업데이트 (Update)
        float y = newAngle - angle; // 혁신(Innovation): 측정값과 예측값의 차이
        float S = P[0][0] + R_measure;
        float K[2]; // 칼만 게인
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;

        angle += K[0] * y;
        bias  += K[1] * y;

        float P00_temp = P[0][0];
        float P01_temp = P[0][1];

        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;

        return angle;
    };

    // 튜닝 파라미터 (여기서 필터 특성 조절)
    // Q_angle을 키우면: 반응 빨라짐 (가속도 더 신뢰)
    // R_measure를 키우면: 더 부드러워짐 (가속도 노이즈 무시)
    void setTuning(float Q_ang, float Q_b, float R_m) {
        Q_angle = Q_ang;
        Q_bias = Q_b;
        R_measure = R_m;
    }

private:
    float Q_angle = 0.001f;   
    float Q_bias  = 0.003f;   
    float R_measure = 0.03f;  
    float angle, bias, rate;
    float P[2][2];
};

Kalman kalmanX;
Kalman kalmanY;

// ==========================================================
// 2. 센서 및 설정
// ==========================================================
#define SPI_CS 10
ICM42670 IMU(SPI, SPI_CS);

// 스케일링 팩터
const float GYRO_SCALE = 1.0 / 16.4;    // 2000dps
const float ACCEL_SCALE = 1.0 / 2048.0; // 16G

unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // SPI 시작
  SPI.begin(12, 13, 11, 10); // SCK, MISO, MOSI, CS

  // IMU 초기화
  if (IMU.begin() < 0) {
    while (1) {
      Serial.println("❌ IMU 연결 실패!");
      delay(1000);
    }
  }

  // 센서 설정 (고속 모드)
  IMU.startAccel(1600, 16); 
  IMU.startGyro(1600, 2000);

  Serial.println("✅ 필터 테스트 시작 (모터 구동 없음)");
  Serial.println("Raw_Accel_Roll,Kalman_Roll,Raw_Accel_Pitch,Kalman_Pitch");
  
  lastTime = micros();
}

void loop() {
  inv_imu_sensor_event_t imu_event;
  IMU.getDataFromRegisters(imu_event);

  // 1. dt 계산 (마이크로초 -> 초)
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;
  if (dt > 0.1) dt = 0.001; // 첫 루프 예외처리
  lastTime = currentTime;

  // 2. Raw 데이터 읽기
  float raw_ax = imu_event.accel[0] * ACCEL_SCALE;
  float raw_ay = imu_event.accel[1] * ACCEL_SCALE;
  float raw_az = imu_event.accel[2] * ACCEL_SCALE;

  float raw_gx = imu_event.gyro[0] * GYRO_SCALE;
  float raw_gy = imu_event.gyro[1] * GYRO_SCALE;

  // 3. 가속도 기반 각도 계산 (노이즈가 심한 원본)
  // atan2를 써서 -180 ~ 180도 범위 계산
  float accAngleX = atan2(raw_ay, sqrt(raw_ax * raw_ax + raw_az * raw_az)) * 180 / PI;
  float accAngleY = atan2(-raw_ax, sqrt(raw_ay * raw_ay + raw_az * raw_az)) * 180 / PI;

  // 4. 칼만 필터 적용 (마법의 시간)
  // 입력: (가속도 각도, 자이로 각속도, 시간간격)
  float kalAngleX = kalmanX.getAngle(accAngleX, raw_gx, dt);
  float kalAngleY = kalmanY.getAngle(accAngleY, raw_gy, dt);

  // 5. 시리얼 플로터 출력
  // 비교 포인트: accAngle(원본)이 튈 때 kalAngle(필터)이 안 튀는지 확인
  Serial.print("AccRoll:"); Serial.print(accAngleX);
  Serial.print(",");
  Serial.print("KalRoll:"); Serial.print(kalAngleX);
  Serial.print(",");
  Serial.print("AccPitch:"); Serial.print(accAngleY);
  Serial.print(",");
  Serial.print("KalPitch:"); Serial.println(kalAngleY);

  // 너무 빠르면 시리얼 플로터가 못 따라가니 살짝 딜레이
  delay(20); 
}