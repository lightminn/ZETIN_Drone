#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "../lib/sensor.h"
#include "../lib/connect.h"

//--- ★★★★★ 사용자 설정 부분 ★★★★★ ---
const char *ssid = "zetin";         // 본인의 스마트폰 핫스팟 이름
const char *password = "nitez348*"; // 핫스팟 비밀번호
const int TCP_PORT = 8888;          // 통신할 포트 번호
//--- 설정 끝 ---

WiFiServer server(TCP_PORT); // TCP 서버 객체 생성
Adafruit_MPU6050 mpu;

// 상보 필터 계수
float COMPLEMENTARY_FILTER_ALPHA = 0.98;

// 자세 각도 변수
float angle_roll = 0;
float angle_pitch = 0;
float angle_yaw = 0;

// 시간 제어 변수
unsigned long loop_timer;
float dt;


void setup()
{
  Serial.begin(115200);
  // MPU6500 초기화 건너뛰기 (센서 없이 테스트용)
  Serial.println("MPU6500 초기화 건너뜀 (센서 없이 테스트 모드).");

  // 센서 범위 설정 (실제 센서가 없으므로 의미는 없지만, 코드 구조 유지를 위해 남겨둠)
  // mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  // mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  delay(100);

  // 첫 루프의 dt 계산을 위한 타이머 초기화
  loop_timer = micros();

  // CSV 출력 헤더
  Serial.println("Roll,Pitch,Yaw");

  delay(1000);

  // Wi-Fi 연결
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" CONNECTED!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // TCP 서버 시작
  server.begin();
  Serial.printf("TCP Server started on port %d\n", TCP_PORT);
}

void loop()
{
  updateSensor();

  // 접속한 클라이언트가 있는지 확인
  WiFiClient client = server.available();

  if (client)
  {
    Serial.println("New client connected!");
    // 클라이언트가 연결되어 있는 동안 반복
    while (client.connected())
    {
      // 클라이언트로부터 데이터가 도착했는지 확인
      if (client.available())
      {
        String line = client.readStringUntil('\n');
        Serial.print("Received command: ");
        Serial.println(line);

        // 클라이언트에게 응답 보내기
        client.printf("OK, Command Received: %s\n", line.c_str());

        // 하나의 명령만 처리하고 루프 탈출
        break;
      }
    }
    // 클라이언트와의 연결 종료
    client.stop();
    Serial.println("Client disconnected.");
  }
  
  // 결과 출력
    Serial.print(angle_roll);
    Serial.print(",");
    Serial.print(angle_pitch);
    Serial.print(",");
    Serial.println(angle_yaw);

    delay(10);
}
