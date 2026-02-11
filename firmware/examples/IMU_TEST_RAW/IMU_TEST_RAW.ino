#include <Arduino.h>
#include <SPI.h>
#include <ICM42670P.h>

// ==========================================
// 핀 설정 (사용하시는 보드에 맞게 확인)
// ==========================================
#define SPI_CS   10
#define SPI_MOSI 11
#define SPI_MISO 13
#define SPI_SCK  12

ICM42670 IMU(SPI, SPI_CS);

void setup() {
  Serial.begin(115200); // 속도 115200
  while (!Serial);

  // SPI 초기화
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);

  // IMU 초기화
  int ret = IMU.begin();
  if (ret != 0) {
    while (1) {
      Serial.println("❌ IMU 연결 실패! 배선 확인하세요.");
      delay(1000);
    }
  }

  // 센서 설정 (범위 설정 중요)
  // 가속도: +/- 16G (진동 보려고 넓게 잡음)
  // 자이로: +/- 2000 dps (빠른 회전 보려고 넓게 잡음)
  IMU.startAccel(1600, 16); 
  IMU.startGyro(1600, 2000);

  Serial.println("Roll(g),Pitch(g),Yaw(g),GyroRoll(dps),GyroPitch(dps),GyroYaw(dps)");
}

void loop() {
  inv_imu_sensor_event_t imu_event;
  IMU.getDataFromRegisters(imu_event);

  // 1. 가속도 (단위: g)
  // 센서 원본값 / 2048.0 = g 단위 (16G 모드일 때)
  float ax = imu_event.accel[0] / 2048.0;
  float ay = imu_event.accel[1] / 2048.0;
  float az = imu_event.accel[2] / 2048.0;

  // 2. 자이로 (단위: dps - degree per second)
  // 센서 원본값 / 16.4 = dps 단위 (2000dps 모드일 때)
  float gx = imu_event.gyro[0] / 16.4;
  float gy = imu_event.gyro[1] / 16.4;
  float gz = imu_event.gyro[2] / 16.4;

  // 3. 시리얼 출력 (CSV 포맷)
  // 순서: Accel X, Y, Z, Gyro X, Y, Z
  Serial.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", ax, ay, az, gx, gy, gz);

  delay(100); // 100Hz 정도 (너무 빠르면 눈 아픔)
}