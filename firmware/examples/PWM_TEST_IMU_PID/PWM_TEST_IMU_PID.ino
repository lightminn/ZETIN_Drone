#include <Arduino.h>
#include <SPI.h>
#include <ICM42670P.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <DFRobot_BMM350.h>

// ==========================================================
// 1. 설정
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

// [최적화] 필터 강도 조절
// 가속도(Accel): 진동에 민감하므로 강하게 (0.1)
// 자이로(Gyro): 반응성이 중요하므로 약하게 (0.4 ~ 0.6)
const float LPF_ALPHA_ACC = 0.10; 
const float LPF_ALPHA_GYRO = 0.40; 

// 튜닝 변수
volatile int base_throttle = 1000;
volatile int min_throttle = 1050;
volatile int max_throttle = 1300;

// [PID 게인 최적화 기본값]
// P를 올려서 복원력을 높이고, I를 기본으로 살짝 넣어 흐름 방지
volatile float Kp_Roll = 2.5;  // 2.0 -> 2.5 (반응성 향상)
volatile float Ki_Roll = 0.005; // 기본적으로 아주 약하게 켜둠
volatile float Kd_Roll = 1.2;

volatile float Kp_Pitch = 2.5;
volatile float Ki_Pitch = 0.005;
volatile float Kd_Pitch = 1.2;

volatile float Kp_Yaw = 3.5; 
volatile float Ki_Yaw = 0.0;   
volatile float Kd_Yaw = 0.0; 

volatile bool safety_lock = true;
volatile float pid_pitch = 0;
volatile float pid_roll = 0;
volatile float pid_yaw = 0; 

volatile float targetAngleX = 0.0; 
volatile float targetAngleY = 0.0; 
volatile float targetAngleZ = 0.0; 

float errorSumRoll = 0, errorSumPitch = 0, errorSumYaw = 0;

const float GYRO_SCALE = 1.0 / 16.4;
const float ACCEL_SCALE = 1.0 / 2048.0;

float angleX = 0, angleY = 0, angleZ = 0; 
// [추가] 자이로 필터 변수
float lpf_ax = 0, lpf_ay = 0, lpf_az = 0; 
float lpf_gx = 0, lpf_gy = 0, lpf_gz = 0; 

unsigned long lastTime = 0;

void writeMotor(int pin, int us) {
  us = constrain(us, 1000, 2000); 
  uint32_t duty = (us * 16383) / ESC_PERIOD;
  ledcWrite(pin, duty);
}

void stopMotors() {
  writeMotor(pinM1, 1000);
  writeMotor(pinM2, 1000);
  writeMotor(pinM3, 1000);
  writeMotor(pinM4, 1000);
}

// ==========================================================
// Core 1: PID 태스크 (진동 무시 설정)
// ==========================================================
void pid_task(void *pvParameters) {
  IMU.startAccel(1600, 16);
  IMU.startGyro(1600, 2000);
  
  const unsigned long LOOP_INTERVAL = 1000; 
  unsigned long nextLoopTime = micros();
  lastTime = micros();

  inv_imu_sensor_event_t imu_event;
  IMU.getDataFromRegisters(imu_event);
  
  // LPF 변수 초기화
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

      // 1. 센서 읽기
      IMU.getDataFromRegisters(imu_event);

      float raw_gx = imu_event.gyro[0] * GYRO_SCALE;
      float raw_gy = imu_event.gyro[1] * GYRO_SCALE;
      float raw_gz = imu_event.gyro[2] * GYRO_SCALE;

      float raw_ax = imu_event.accel[0] * ACCEL_SCALE;
      float raw_ay = imu_event.accel[1] * ACCEL_SCALE;
      float raw_az = imu_event.accel[2] * ACCEL_SCALE;

      // [필터 강화] 가속도 LPF를 더 강하게 (0.1 -> 0.05)
      // 반응은 느려지지만 진동 억제력 상승
      lpf_ax = (0.05 * raw_ax) + (0.95 * lpf_ax);
      lpf_ay = (0.05 * raw_ay) + (0.95 * lpf_ay);
      lpf_az = (0.05 * raw_az) + (0.95 * lpf_az);

      // 자이로 필터 (적당히 유지)
      lpf_gx = (0.4 * raw_gx) + (0.6 * lpf_gx);
      lpf_gy = (0.4 * raw_gy) + (0.6 * lpf_gy);
      lpf_gz = (0.4 * raw_gz) + (0.6 * lpf_gz);

      float dt = (currentMicros - lastTime) / 1000000.0;
      if (dt > 0.002) dt = 0.001; 
      lastTime = currentMicros;

      // 2. 자세 추정 (여기가 핵심!)
      float accAngleX = atan2(lpf_ay, sqrt(lpf_ax * lpf_ax + lpf_az * lpf_az)) * 180 / PI;
      float accAngleY = atan2(-lpf_ax, sqrt(lpf_ay * lpf_ay + lpf_az * lpf_az)) * 180 / PI;

      // [핵심 수정] 상보필터 비율 극단적 조정
      // 기존: 0.98 (자이로) vs 0.02 (가속도)
      // 변경: 0.999 (자이로) vs 0.001 (가속도)
      // -> 가속도 센서가 "야 기울어졌어!"라고 거짓말해도 자이로가 "응 아니야 안 돌았어" 하고 무시함.
      angleX = 0.999 * (angleX + lpf_gx * dt) + 0.001 * accAngleX;
      angleY = 0.999 * (angleY + lpf_gy * dt) + 0.001 * accAngleY;
      
      // Yaw
      if (abs(lpf_gz) > 0.3) angleZ += lpf_gz * dt; 

      if (abs(angleX) > 45 || abs(angleY) > 45) {
         safety_lock = true;
      } else {
        float errorRoll = targetAngleX - angleX;
        float errorPitch = targetAngleY - angleY;
        float errorYaw = targetAngleZ - angleZ;

        // 적분 (I항) 로직
        if (base_throttle < 1100) {
          errorSumRoll = 0; errorSumPitch = 0; errorSumYaw = 0;
        } 
        else {
          if (abs(errorRoll) < 25.0)  errorSumRoll  += errorRoll * dt;
          if (abs(errorPitch) < 25.0) errorSumPitch += errorPitch * dt;
          if (abs(errorYaw) < 25.0)   errorSumYaw   += errorYaw * dt;
        }
        
        errorSumRoll  = constrain(errorSumRoll, -15.0, 15.0);
        errorSumPitch = constrain(errorSumPitch, -15.0, 15.0);
        errorSumYaw   = constrain(errorSumYaw, -15.0, 15.0);

        pid_pitch = (errorPitch * Kp_Pitch) + (errorSumPitch * Ki_Pitch) - (lpf_gy * Kd_Pitch);
        pid_roll  = (errorRoll * Kp_Roll)   + (errorSumRoll * Ki_Roll)   - (lpf_gx * Kd_Roll);
        pid_yaw   = (errorYaw * Kp_Yaw)     + (errorSumYaw * Ki_Yaw)     - (lpf_gz * Kd_Yaw);
      }
      
      if (safety_lock) {
        stopMotors();
        errorSumRoll = 0; errorSumPitch = 0; errorSumYaw = 0;
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

      udp.beginPacket(laptopIP, laptopPort);
      
      // 포맷: "Roll,Pitch,Yaw,gyro_Roll,gyro_Pitch,gyro_Yaw,accel_Roll,accel_Pitch,accel_Yaw,Throttle" (CSV 파싱하기 쉽게 콤마로 구분)
      // 예: "12.5,-3.2,140.1,1050"
      udp.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d", angleX, angleY, angleZ, raw_gx, raw_gy, raw_gz, raw_ax, raw_ay, raw_az, base_throttle);
      
      udp.endPacket();

    } else {
      vTaskDelay(0);
    }
  }
}

// UDP 태스크와 Setup, Loop는 그대로 유지 (너무 길어서 생략, 위 코드와 동일)
// ... (udp_task, setup, loop 함수는 기존과 100% 동일하게 사용하세요) ...
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
              String sRoll = cmd.substring(s1 + 1, s2);
              String sPitch = cmd.substring(s2 + 1, s3 > 0 ? s3 : cmd.length());
              
              targetAngleX = sRoll.toFloat();
              targetAngleY = sPitch.toFloat();

              if (s3 > 0) {
                 String sYaw = cmd.substring(s3 + 1);
                 targetAngleZ = sYaw.toFloat();
              }
          }
      } 
      else {
          float val = cmd.substring(2).toFloat();
          
          if (cmd.startsWith("pa")) { Kp_Roll = val; Kp_Pitch = val; }
          else if (cmd.startsWith("da")) { Kd_Roll = val; Kd_Pitch = val; }
          else if (cmd.startsWith("ia")) { Ki_Roll = val; Ki_Pitch = val; }
          
          else if (cmd.startsWith("pp")) { Kp_Pitch = val; }
          else if (cmd.startsWith("dp")) { Kd_Pitch = val; }
          else if (cmd.startsWith("ip")) { Ki_Pitch = val; }
          
          else if (cmd.startsWith("pr")) { Kp_Roll = val; }
          else if (cmd.startsWith("dr")) { Kd_Roll = val; }
          else if (cmd.startsWith("ir")) { Ki_Roll = val; }
          
          else if (cmd.startsWith("py")) { Kp_Yaw = val; }
          else if (cmd.startsWith("dy")) { Kd_Yaw = val; }
          else if (cmd.startsWith("iy")) { Ki_Yaw = val; }
          
          else if (cmd.startsWith("th")) { 
            int new_base = (int)val;
            base_throttle = new_base;
            
            int new_min = new_base - CTRL_MARGIN;
            int new_max = new_base + CTRL_MARGIN;

            if (new_min < 1050) new_min = 1050; 
            if (new_max > 1900) new_max = 1900; 

            min_throttle = new_min;
            max_throttle = new_max;
          }
          else if (cmd.startsWith("start")) { 
            safety_lock = false; 
            base_throttle = 1100;
            min_throttle = 1050; 
            max_throttle = 1250; 
            targetAngleX = 0; targetAngleY = 0; targetAngleZ = 0;
            angleZ = 0; 
            errorSumRoll = 0; errorSumPitch = 0; errorSumYaw = 0;
          }
          else if (cmd.startsWith("stop")) { 
            safety_lock = true; 
            base_throttle = 1000; 
          }
      }
    }
    vTaskDelay(2);
  }
}

void setup() {
  Serial.begin(115200);
  
  WiFi.softAP(ssid, password);
  Serial.println("WiFi Ready");
  udp.begin(udpPort);

  SPI.begin(12, 13, 11, 10);

  if (!ledcAttach(pinM1, ESC_FREQ, ESC_RES)) Serial.println("M1 Attach Fail");
  if (!ledcAttach(pinM2, ESC_FREQ, ESC_RES)) Serial.println("M2 Attach Fail");
  if (!ledcAttach(pinM3, ESC_FREQ, ESC_RES)) Serial.println("M3 Attach Fail");
  if (!ledcAttach(pinM4, ESC_FREQ, ESC_RES)) Serial.println("M4 Attach Fail");

  stopMotors(); 

  if (IMU.begin() < 0) {
    while (1) { Serial.println("IMU Fail"); delay(1000); }
  }
  delay(1000);

  xTaskCreatePinnedToCore(pid_task, "PID", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(udp_task, "UDP", 4096, NULL, 0, NULL, 0);

  Serial.println("SYSTEM READY (Optimized LPF + Deadband)");
}

void loop() {
  // static unsigned long lastSendTime = 0;
  
  // // 0.05초(50ms)마다 전송 (너무 빠르면 파이썬 그래프 렉걸림)
  // if (millis() - lastSendTime > 50) { 
  //   lastSendTime = millis();

  //   // 노트북이랑 연결된 상태라면 데이터 전송
  //   if (connectionEstablished) {
  //     udp.beginPacket(laptopIP, laptopPort);
      
  //     // 포맷: "Roll,Pitch,Yaw,Throttle" (CSV 파싱하기 쉽게 콤마로 구분)
  //     // 예: "12.5,-3.2,140.1,1050"
  //     udp.printf("%.2f,%.2f,%.2f,%d", angleX, angleY, angleZ, base_throttle);
      
  //     udp.endPacket();
  //   }
  // }
}