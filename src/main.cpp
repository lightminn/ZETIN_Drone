#include <Arduino.h>
#include <WiFi.h>

//--- ★★★★★ 사용자 설정 부분 ★★★★★ ---
const char *ssid = "zetin";         // 본인의 스마트폰 핫스팟 이름
const char *password = "nitez348*"; // 핫스팟 비밀번호
const int TCP_PORT = 8888;          // 통신할 포트 번호
//--- 설정 끝 ---

WiFiServer server(TCP_PORT); // TCP 서버 객체 생성

void setup()
{
  Serial.begin(115200);
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
}