#include <Arduino.h>
#include <SPI.h>
#include <ICM42670P.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ==========================================================
// 1. 칼만 필터 클래스 정의
// ==========================================================
class Kalman {
public:
  Kalman() {
    P[0][0] = 0.1f; P[0][1] = 0.0f;
    P[1][0] = 0.0f; P[1][1] = 0.1f;
    angle = 0.0f;
    bias = 0.0f;
    }

    // newAngle: 가속도계 각도, newRate: 자이로 각속도, dt: 시간간격
    float getAngle(float newAngle, float newRate, float dt) {
        // 1. 예측 (Predict)
        rate = newRate - bias;
        angle += dt * rate;

        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;

        // 2. 업데이트 (Update)
        float y = newAngle - angle;
        float S = P[0][0] + R_measure;
        float K[2]; // Kalman Gain
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

private:
    // [칼만 튜닝 파라미터]
    // Q_angle: 가속도계 신뢰도 (높으면 반응 빠름, 낮으면 부드러움)
    float Q_angle = 0.005f;   
    // Q_bias: 자이로 드리프트 추정 속도
    float Q_bias  = 0.02f;   
    // R_measure: 측정 노이즈 공분산 (높으면 가속도 노이즈 무시)
    float R_measure = 0.01f;  
    
    float angle, bias, rate;
    float P[2][2];
};

// 칼만 필터 인스턴스 생성
Kalman kalmanX;
Kalman kalmanY;

// ==========================================================
// 2. 설정 (튜닝 변수)
// ==========================================================
// [필터 강도 설정] 
// 칼만 필터를 쓰더라도 가속도 원본 노이즈가 너무 심하면 안 좋으므로
// 약한 LPF를 거쳐서 칼만에 넣어주는 것이 좋습니다.
const float LPF_ALPHA_ACC = 1.f; // 가속도 LPF
const float LPF_ALPHA_GYRO = 1.f; // 자이로 LPF

// PID 게인
volatile float Kp_Roll = 2.5,  Ki_Roll = 0.005, Kd_Roll = 1.2;
volatile float Kp_Pitch = 2.5, Ki_Pitch = 0.005, Kd_Pitch = 1.2;
volatile float Kp_Yaw = 3.5,   Ki_Yaw = 0.0,    Kd_Yaw = 0.0; 

// 스로틀 설정
volatile int base_throttle = 1000;
volatile int min_throttle = 1050;
volatile int max_throttle = 1300;

// ==========================================================
// 3. 시스템 변수
// ==========================================================
const char *ssid = "Drone_Tuning";
const char *password = "12345678";
WiFiUDP udp;
const int udpPort = 4210;
char packetBuffer[255];

IPAddress laptopIP;
int laptopPort = 0;
bool connectionEstablished = false;

const int pinM1 = 4; // RR (CW)
const int pinM2 = 5; // FL (CW)
const int pinM3 = 6; // RL (CCW)
const int pinM4 = 7; // FR (CCW)
#define SPI_CS 10

ICM42670 IMU(SPI, SPI_CS);

const int ESC_FREQ = 400; 
const int ESC_RES = 14;   
const int ESC_PERIOD = 2500; 

volatile bool safety_lock = true;
volatile float targetAngleX = 0.0, targetAngleY = 0.0, targetAngleZ = 0.0; 
float errorSumRoll = 0, errorSumPitch = 0, errorSumYaw = 0;

// 전역 변수 (Loop 전송용)
volatile float raw_gx = 0, raw_gy = 0, raw_gz = 0;
volatile float raw_ax = 0, raw_ay = 0, raw_az = 0;
float angleX = 0, angleY = 0, angleZ = 0; 

// 내부 계산용 필터 변수
float lpf_ax = 0, lpf_ay = 0, lpf_az = 0; 
float lpf_gx = 0, lpf_gy = 0, lpf_gz = 0; 

// 스케일링 팩터
const float GYRO_SCALE = 1.0 / 16.4;    
const float ACCEL_SCALE = 1.0 / 2048.0; 

void writeMotor(int pin, int us) {
  us = constrain(us, 1000, 2000); 
  ledcWrite(pin, (us * 16383) / ESC_PERIOD);
}

void stopMotors() {
  writeMotor(pinM1, 1000); writeMotor(pinM2, 1000);
  writeMotor(pinM3, 1000); writeMotor(pinM4, 1000);
}

// ==========================================================
// [Core 1] PID 태스크 (칼만 필터 적용됨)
// ==========================================================
void pid_task(void *pvParameters) {
  IMU.startAccel(1600, 16);   
  IMU.startGyro(1600, 2000);  
  
  const unsigned long LOOP_INTERVAL = 1000; 
  unsigned long nextLoopTime = micros();
  unsigned long lastTime = micros();

  inv_imu_sensor_event_t imu_event;
  
  // 초기값 로드
  IMU.getDataFromRegisters(imu_event);
  lpf_ax = imu_event.accel[0] * ACCEL_SCALE;
  lpf_ay = imu_event.accel[1] * ACCEL_SCALE;
  lpf_az = imu_event.accel[2] * ACCEL_SCALE;
  lpf_gx = imu_event.gyro[0] * GYRO_SCALE;
  lpf_gy = imu_event.gyro[1] * GYRO_SCALE;
  lpf_gz = imu_event.gyro[2] * GYRO_SCALE;

  while (true) {
    unsigned long currentMicros = micros();

    if (currentMicros >= nextLoopTime) {
      nextLoopTime = currentMicros + LOOP_INTERVAL;
      
      // 1. 센서 읽기 & 전역 변수 업데이트
      IMU.getDataFromRegisters(imu_event);

      raw_gx = imu_event.gyro[0] * GYRO_SCALE;
      raw_gy = imu_event.gyro[1] * GYRO_SCALE;
      raw_gz = imu_event.gyro[2] * GYRO_SCALE;

      raw_ax = imu_event.accel[0] * ACCEL_SCALE;
      raw_ay = imu_event.accel[1] * ACCEL_SCALE;
      raw_az = imu_event.accel[2] * ACCEL_SCALE;

      // 2. 입력 데이터 전처리 (LPF)
      // 칼만 필터에 넣기 전에 가속도 노이즈를 1차로 걸러줍니다.
      lpf_ax = (LPF_ALPHA_ACC * raw_ax) + ((1.0f - LPF_ALPHA_ACC) * lpf_ax);
      lpf_ay = (LPF_ALPHA_ACC * raw_ay) + ((1.0f - LPF_ALPHA_ACC) * lpf_ay);
      lpf_az = (LPF_ALPHA_ACC * raw_az) + ((1.0f - LPF_ALPHA_ACC) * lpf_az);

      // 자이로도 LPF 처리 (D항 및 칼만 입력용)
      lpf_gx = (LPF_ALPHA_GYRO * raw_gx) + ((1.0f - LPF_ALPHA_GYRO) * lpf_gx);
      lpf_gy = (LPF_ALPHA_GYRO * raw_gy) + ((1.0f - LPF_ALPHA_GYRO) * lpf_gy);
      lpf_gz = (LPF_ALPHA_GYRO * raw_gz) + ((1.0f - LPF_ALPHA_GYRO) * lpf_gz);

      float dt = (currentMicros - lastTime) / 1000000.0;
      if (dt > 0.005) dt = 0.001; // 예외처리
      lastTime = currentMicros;

      // 3. 자세 추정 (Kalman Filter)
      // 가속도 기반 각도 (atan2)
      float accAngleX = atan2(lpf_ay, sqrt(lpf_ax * lpf_ax + lpf_az * lpf_az)) * 180 / PI;
      float accAngleY = atan2(-lpf_ax, sqrt(lpf_ay * lpf_ay + lpf_az * lpf_az)) * 180 / PI;

      // [핵심] 상보필터 대신 칼만 필터 사용
      // lpf_gx를 넣으면 조금 더 부드럽고, raw_gx를 넣으면 더 빠릅니다. (여기선 안전하게 lpf 사용)
      angleX = kalmanX.getAngle(accAngleX, lpf_gx, dt);
      angleY = kalmanY.getAngle(accAngleY, lpf_gy, dt);
      
      // Yaw는 자이로 적분만
      if (abs(lpf_gz) > 0.3) angleZ += lpf_gz * dt; 

      // 4. PID 제어
      if (abs(angleX) > 45 || abs(angleY) > 45) {
         safety_lock = true;
      } 
      else {
        float errorRoll = targetAngleX - angleX;
        float errorPitch = targetAngleY - angleY;
        float errorYaw = targetAngleZ - angleZ;

        // I항 처리
        if (base_throttle < 1100) {
          errorSumRoll = 0; errorSumPitch = 0; errorSumYaw = 0;
        } else {
          if (abs(errorRoll) < 25.0)  errorSumRoll  += errorRoll * dt;
          if (abs(errorPitch) < 25.0) errorSumPitch += errorPitch * dt;
          if (abs(errorYaw) < 25.0)   errorSumYaw   += errorYaw * dt;
        }
        
        errorSumRoll  = constrain(errorSumRoll, -10.0, 10.0);
        errorSumPitch = constrain(errorSumPitch, -10.0, 10.0);
        errorSumYaw   = constrain(errorSumYaw, -10.0, 10.0);

        // PID 출력
        float pid_pitch = (errorPitch * Kp_Pitch) + (errorSumPitch * Ki_Pitch) - (lpf_gy * Kd_Pitch);
        float pid_roll  = (errorRoll * Kp_Roll)   + (errorSumRoll * Ki_Roll)   - (lpf_gx * Kd_Roll);
        float pid_yaw   = (errorYaw * Kp_Yaw)     + (errorSumYaw * Ki_Yaw)     - (lpf_gz * Kd_Yaw);

        // 모터 믹싱
        if (safety_lock) {
          stopMotors();
        } else {
          int pwm1 = base_throttle - pid_pitch - pid_roll + pid_yaw; 
          int pwm2 = base_throttle + pid_pitch + pid_roll + pid_yaw; 
          int pwm3 = base_throttle - pid_pitch + pid_roll - pid_yaw;  
          int pwm4 = base_throttle + pid_pitch - pid_roll - pid_yaw; 

          writeMotor(pinM1, constrain(pwm1, min_throttle, max_throttle));
          writeMotor(pinM2, constrain(pwm2, min_throttle, max_throttle));
          writeMotor(pinM3, constrain(pwm3, min_throttle, max_throttle));
          writeMotor(pinM4, constrain(pwm4, min_throttle, max_throttle));
        }
      }
    } else {
      vTaskDelay(0);
    }
  }
}

// ==========================================================
// [Core 0] UDP 통신 태스크 (동일)
// ==========================================================
void udp_task(void *pvParameters) {
  const int CTRL_MARGIN = 150; 
  while (true) {
    int packetSize = udp.parsePacket();
    if (packetSize) {
      laptopIP = udp.remoteIP();
      laptopPort = udp.remotePort();
      connectionEstablished = true;
      int len = udp.read(packetBuffer, 255);
      if (len > 0) packetBuffer[len] = 0;
      String cmd = String(packetBuffer);
      cmd.trim();

      if (cmd.startsWith("rc")) {
          int s1 = cmd.indexOf(' ');
          int s2 = cmd.indexOf(' ', s1 + 1);
          int s3 = cmd.indexOf(' ', s2 + 1);
          if (s1 > 0 && s2 > 0) {
              targetAngleX = cmd.substring(s1 + 1, s2).toFloat();
              targetAngleY = cmd.substring(s2 + 1, s3 > 0 ? s3 : cmd.length()).toFloat();
              if (s3 > 0) targetAngleZ = cmd.substring(s3 + 1).toFloat();
          }
      } 
      else {
          float val = cmd.substring(2).toFloat();
          if (cmd.startsWith("pa")) { Kp_Roll = val; Kp_Pitch = val; }
          else if (cmd.startsWith("da")) { Kd_Roll = val; Kd_Pitch = val; }
          else if (cmd.startsWith("ia")) { Ki_Roll = val; Ki_Pitch = val; }
          else if (cmd.startsWith("pr")) { Kp_Roll = val; } // ... 생략 (이전과 동일)
          else if (cmd.startsWith("th")) { 
            int new_base = (int)val;
            base_throttle = new_base;
            min_throttle = max(1050, new_base - CTRL_MARGIN);
            max_throttle = min(1900, new_base + CTRL_MARGIN);
          }
          else if (cmd.startsWith("start")) { 
            safety_lock = false; 
            base_throttle = 1100; min_throttle = 1050; max_throttle = 1250; 
            targetAngleX = 0; targetAngleY = 0; targetAngleZ = 0;
            angleZ = 0; errorSumRoll = 0; errorSumPitch = 0; errorSumYaw = 0;
          }
          else if (cmd.startsWith("stop")) { 
            safety_lock = true; base_throttle = 1000; 
          }
      }
    }
    vTaskDelay(2);
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.softAP(ssid, password);
  udp.begin(udpPort);
  SPI.begin(12, 13, 11, 10);
  ledcAttach(pinM1, ESC_FREQ, ESC_RES);
  ledcAttach(pinM2, ESC_FREQ, ESC_RES);
  ledcAttach(pinM3, ESC_FREQ, ESC_RES);
  ledcAttach(pinM4, ESC_FREQ, ESC_RES);
  stopMotors(); 
  if (IMU.begin() < 0) { while (1) { Serial.println("IMU Fail"); delay(1000); } }
  delay(1000);
  xTaskCreatePinnedToCore(pid_task, "PID", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(udp_task, "UDP", 4096, NULL, 0, NULL, 0);
  Serial.println("SYSTEM READY (Kalman Filter Active)");
}

void loop() {
  // static unsigned long lastSendTime = 0;
  // // [중요 수정] 1ms -> 50ms (20Hz)
  // // 너무 빠르면 네트워크가 막혀서 드론 제어가 렉걸립니다.
  // if (millis() - lastSendTime > 10) { 
  //   lastSendTime = millis();
  //   if (connectionEstablished) {
  //     udp.beginPacket(laptopIP, laptopPort);
  //     udp.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d", 
  //                angleX, angleY, angleZ, 
  //                raw_gx, raw_gy, raw_gz, 
  //                raw_ax, raw_ay, raw_az, 
  //                base_throttle);
  //     udp.endPacket();
  //   }
  // }
}