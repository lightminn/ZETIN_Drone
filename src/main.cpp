#include <Arduino.h>
#include <WiFi.h>
#include <AsyncUDP.h>
#include <ESPmDNS.h> // mDNS 라이브러리
#include <mavlink.h> // MAVLink 라이브러리

//--- 사용자 설정 ---
const char *ssid = "rspi";
const char *password = "123454321";
const int UDP_PORT = 14550; // MAVLink 표준 포트
//--- 설정 끝 ---

AsyncUDP udp;

void setup()
{
  Serial.begin(115200);
  delay(1000);

  // 1. Wi-Fi 연결
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

  // 2. mDNS 시작: 이제 'http://drone-fc.local'로 찾을 수 있음
  if (MDNS.begin("drone-fc"))
  {
    Serial.println("MDNS responder started. Hostname: drone-fc.local");
  }

  // 3. UDP 리스너 설정
  if (udp.listen(UDP_PORT))
  {
    Serial.print("UDP Listening on port: ");
    Serial.println(UDP_PORT);

    udp.onPacket([](AsyncUDPPacket packet)
                 {
      mavlink_message_t msg;
      mavlink_status_t status;

      // 수신된 패킷을 한 바이트씩 MAVLink 파서에 넣어줌
      for (int i = 0; i < packet.length(); i++) {
        if (mavlink_parse_char(MAVLINK_COMM_0, packet.data()[i], &msg, &status)) {
          // 완전한 MAVLink 메시지가 수신되면 여기로 들어옴
          switch (msg.msgid) {
            case MAVLINK_MSG_ID_MANUAL_CONTROL:
              {
                mavlink_manual_control_t manual_control;
                mavlink_msg_manual_control_decode(&msg, &manual_control);
                Serial.println("--- MAVLink MANUAL_CONTROL Received! ---");
                Serial.printf("Throttle (z): %d\n", manual_control.z);
                Serial.printf("Roll (y): %d\n", manual_control.y);
                Serial.printf("Pitch (x): %d\n", manual_control.x);
                Serial.printf("Yaw (r): %d\n", manual_control.r);
                Serial.println("----------------------------------------");
              }
              break;
            // 다른 MAVLink 메시지를 처리하고 싶으면 여기에 추가
            // case MAVLINK_MSG_ID_... :
          }
        }
      } });
  }
}

void loop()
{
  // 모든 작업은 비동기 콜백에서 처리됨
}