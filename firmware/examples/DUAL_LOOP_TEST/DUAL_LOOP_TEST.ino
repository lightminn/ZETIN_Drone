#include <Arduino.h>
#include <SPI.h>
#include <ICM42670P.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ==========================================================
// 1. 유틸리티 클래스 (Flix 스타일 LPF & PID)
// ==========================================================

// [Low Pass Filter] 미분항(D) 노이즈 제거용
class LowPassFilter {
public:
    float alpha;
    float lastOutput = 0;

    LowPassFilter(float cutoff_freq, float dt) {
        float rc = 1.0f / (2.0f * PI * cutoff_freq);
        alpha = dt / (rc + dt);
    }

    float update(float input) {
        lastOutput = lastOutput + alpha * (input - lastOutput);
        return lastOutput;
    }
};

// ==========================================================
// 2. 튜닝 변수 (이중 루프 구조)
// ==========================================================
// [Outer Loop] 각도 제어 (P gain만 사용 - Flix 방식)
// 의미: 각도 오차가 10도일 때, 초당 몇 도(dps)로 회전하라고 시킬 것인가?
volatile float Kp_Angle_Roll = 6.0;
volatile float Kp_Angle_Pitch = 6.0;
volatile float Kp_Angle_Yaw = 3.0; // Yaw는 각도 제어보다 속도 제어가 메인

// [Inner Loop] 각속도(Rate) 제어 (PID 사용)
// 의미: 목표 회전 속도를 맞추기 위해 모터를 얼마나 세게 돌릴 것인가?
volatile float Kp_Rate_Roll = 0.5;
volatile float Ki_Rate_Roll = 0.005;
volatile float Kd_Rate_Roll = 0.015;

volatile float Kp_Rate_Pitch = 0.5;
volatile float Ki_Rate_Pitch = 0.005;
volatile float Kd_Rate_Pitch = 0.015;

volatile float Kp_Rate_Yaw = 1.5;
volatile float Ki_Rate_Yaw = 0.05;
volatile float Kd_Rate_Yaw = 0.0;

// [스마트 가속도] 가속도 벡터 크기가 이 범위 안일 때만 자세 보정 (1G = 1.0)
const float ACC_MARGIN = 0.15; // 0.85G ~ 1.15G 사이일 때만 신뢰

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

const int pinM1 = 4; const int pinM2 = 5;
const int pinM3 = 6; const int pinM4 = 7;
#define SPI_CS 10
ICM42670 IMU(SPI, SPI_CS);

const int ESC_FREQ = 400; const int ESC_RES = 14; const int ESC_PERIOD = 2500;

volatile bool safety_lock = true;
volatile float targetAngleX = 0.0, targetAngleY = 0.0, targetAngleZ = 0.0;

// 전역 변수 (모니터링용)
float angleX = 0, angleY = 0, angleZ = 0; // 현재 각도
float gyroX = 0, gyroY = 0, gyroZ = 0;    // 현재 각속도 (deg/s)

// 적분 누적 (Rate Loop용)
float errorSumRateRoll = 0, errorSumRatePitch = 0, errorSumRateYaw = 0;

// 스케일링
const float GYRO_SCALE = 1.0 / 16.4;     // raw -> deg/s
const float ACCEL_SCALE = 1.0 / 2048.0;  // raw -> g

void writeMotor(int pin, int us) {
    us = constrain(us, 1000, 2000);
    ledcWrite(pin, (us * 16383) / ESC_PERIOD);
}
void stopMotors() {
    writeMotor(pinM1, 1000); writeMotor(pinM2, 1000);
    writeMotor(pinM3, 1000); writeMotor(pinM4, 1000);
}

// ==========================================================
// [Core 1] PID 태스크 (Flix 알고리즘 적용)
// ==========================================================
void pid_task(void *pvParameters) {
    IMU.startAccel(1600, 16);
    IMU.startGyro(1600, 2000);

    const unsigned long LOOP_INTERVAL = 1000; // 1kHz Loop
    unsigned long nextLoopTime = micros();
    unsigned long lastTime = micros();
    float dt = 0.001;

    inv_imu_sensor_event_t imu_event;

    // D항 노이즈 제거용 LPF (Cutoff 40Hz)
    LowPassFilter lpfD_Roll(40, dt);
    LowPassFilter lpfD_Pitch(40, dt);

    // 미분 계산용 이전 오차
    float prevErrorRateRoll = 0, prevErrorRatePitch = 0;

    while (true) {
        unsigned long currentMicros = micros();

        if (currentMicros >= nextLoopTime) {
            dt = (currentMicros - lastTime) / 1000000.0;
            if (dt > 0.002) dt = 0.001;
            lastTime = currentMicros;
            nextLoopTime = currentMicros + LOOP_INTERVAL;

            // 1. 센서 읽기
            IMU.getDataFromRegisters(imu_event);
            float ax = imu_event.accel[0] * ACCEL_SCALE;
            float ay = imu_event.accel[1] * ACCEL_SCALE;
            float az = imu_event.accel[2] * ACCEL_SCALE;

            // 자이로: LPF 없이 원본 사용 (Inner Loop 반응성 위해)
            gyroX = imu_event.gyro[0] * GYRO_SCALE;
            gyroY = imu_event.gyro[1] * GYRO_SCALE;
            gyroZ = imu_event.gyro[2] * GYRO_SCALE;

            // 2. [Flix 스타일] 스마트 자세 추정
            // 자이로 적분 (기본)
            angleX += gyroX * dt;
            angleY += gyroY * dt;

            // Yaw는 적분만 (지자기 없음)
            if (abs(gyroZ) > 0.3) angleZ += gyroZ * dt;

            // 가속도 보정 (중력 가속도 1G 근처일 때만 신뢰)
            float accMagnitude = sqrt(ax*ax + ay*ay + az*az);
            if (abs(accMagnitude - 1.0) < ACC_MARGIN) {
                // 가속도 각도 계산
                float accAngleX = atan2(ay, sqrt(ax*ax + az*az)) * 180 / PI;
                float accAngleY = atan2(-ax, sqrt(ay*ay + az*az)) * 180 / PI;

                // 상보필터 (가속도 비중 0.5% - 아주 조금씩 보정)
                angleX = angleX * 0.995 + accAngleX * 0.005;
                angleY = angleY * 0.995 + accAngleY * 0.005;
            }

            // ----------------------------------------------------
            // 3. [이중 루프 PID]
            // ----------------------------------------------------

            if (abs(angleX) > 60 || abs(angleY) > 60) safety_lock = true;

            if (!safety_lock) {
                // [Outer Loop] Angle Control (P Controller)
                // 목표: 각도 오차 -> 목표 회전 속도(Target Rate)
                float errorAngleRoll  = targetAngleX - angleX;
                float errorAnglePitch = targetAngleY - angleY;
                float errorAngleYaw   = targetAngleZ - angleZ;

                float targetRateRoll  = errorAngleRoll  * Kp_Angle_Roll;
                float targetRatePitch = errorAnglePitch * Kp_Angle_Pitch;
                float targetRateYaw   = errorAngleYaw   * Kp_Angle_Yaw;

                // [Inner Loop] Rate Control (PID Controller)
                // 목표: 회전 속도 오차 -> 모터 파워
                float errorRateRoll  = targetRateRoll  - gyroX;
                float errorRatePitch = targetRatePitch - gyroY;
                float errorRateYaw   = targetRateYaw   - gyroZ;

                // I항 (Anti-Windup 적용)
                if (base_throttle > 1100) {
                    errorSumRateRoll  = constrain(errorSumRateRoll  + errorRateRoll  * dt, -200, 200);
                    errorSumRatePitch = constrain(errorSumRatePitch + errorRatePitch * dt, -200, 200);
                    errorSumRateYaw   = constrain(errorSumRateYaw   + errorRateYaw   * dt, -200, 200);
                } else {
                    errorSumRateRoll = 0; errorSumRatePitch = 0; errorSumRateYaw = 0;
                }

                // D항 (LPF 적용 - 진동 방지 핵심)
                float dTermRoll = (errorRateRoll - prevErrorRateRoll) / dt;
                dTermRoll = lpfD_Roll.update(dTermRoll); // 필터링!

                float dTermPitch = (errorRatePitch - prevErrorRatePitch) / dt;
                dTermPitch = lpfD_Pitch.update(dTermPitch); // 필터링!

                prevErrorRateRoll = errorRateRoll;
                prevErrorRatePitch = errorRatePitch;

                // 최종 PID 계산
                float pidRoll  = (errorRateRoll * Kp_Rate_Roll)   + (errorSumRateRoll * Ki_Rate_Roll)   + (dTermRoll * Kd_Rate_Roll);
                float pidPitch = (errorRatePitch * Kp_Rate_Pitch) + (errorSumRatePitch * Ki_Rate_Pitch) + (dTermPitch * Kd_Rate_Pitch);
                float pidYaw   = (errorRateYaw * Kp_Rate_Yaw)     + (errorSumRateYaw * Ki_Rate_Yaw);    // Yaw는 D항 보통 안 씀

                // 모터 믹싱
                int pwm1 = base_throttle - pidPitch - pidRoll + pidYaw;
                int pwm2 = base_throttle + pidPitch + pidRoll + pidYaw;
                int pwm3 = base_throttle - pidPitch + pidRoll - pidYaw;
                int pwm4 = base_throttle + pidPitch - pidRoll - pidYaw;

                writeMotor(pinM1, constrain(pwm1, min_throttle, max_throttle));
                writeMotor(pinM2, constrain(pwm2, min_throttle, max_throttle));
                writeMotor(pinM3, constrain(pwm3, min_throttle, max_throttle));
                writeMotor(pinM4, constrain(pwm4, min_throttle, max_throttle));
            } else {
                stopMotors();
            }

        } else {
            vTaskDelay(0);
        }
    }
}

// ==========================================================
// [Core 0] UDP 통신 태스크 (명령어 파싱 수정)
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
                if (s1 > 0 && s2 > 0) {
                    targetAngleX = cmd.substring(s1 + 1, s2).toFloat();
                    targetAngleY = cmd.substring(s2 + 1).toFloat();
                }
            }
            // 튜닝 명령어 (Rate PID 튜닝 위주로 변경)
            else {
                float val = cmd.substring(2).toFloat();
                // Rate P, I, D 튜닝
                if (cmd.startsWith("rp")) { Kp_Rate_Roll = val; Kp_Rate_Pitch = val; }
                else if (cmd.startsWith("rd")) { Kd_Rate_Roll = val; Kd_Rate_Pitch = val; }
                else if (cmd.startsWith("ri")) { Ki_Rate_Roll = val; Ki_Rate_Pitch = val; }
                // Angle P 튜닝
                else if (cmd.startsWith("ap")) { Kp_Angle_Roll = val; Kp_Angle_Pitch = val; }

                else if (cmd.startsWith("start")) {
                    safety_lock = false;
                    base_throttle = 1100; min_throttle = 1050; max_throttle = 1250;
                    angleX=0; angleY=0; angleZ=0;
                    errorSumRateRoll=0; errorSumRatePitch=0; errorSumRateYaw=0;
                }
                else if (cmd.startsWith("stop")) {
                    safety_lock = true; base_throttle = 1000;
                }
                else if (cmd.startsWith("th")) {
                    int new_base = (int)val;
                    base_throttle = new_base;
                    min_throttle = max(1050, new_base - CTRL_MARGIN);
                    max_throttle = min(1900, new_base + CTRL_MARGIN);
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
    Serial.println("SYSTEM READY (Dual Loop PID)");
}

void loop() {
    static unsigned long lastSendTime = 0;
    // 모니터링 데이터 전송 (50ms)
    if (millis() - lastSendTime > 50) {
        lastSendTime = millis();
        if (connectionEstablished) {
            udp.beginPacket(laptopIP, laptopPort);
            udp.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d",
                       angleX, angleY, angleZ,
                       gyroX, gyroY, gyroZ,   // Raw gyro
                       0.0, 0.0, 0.0,         // Accel은 굳이 안 보내도 됨
                       base_throttle);
            udp.endPacket();
        }
    }
}
