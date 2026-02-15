#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ==========================================
// 설정 (PCB 핀 번호)
// ==========================================
// 43, 44번 핀 고정
#define GPS_RX_PIN 44  // ESP32 RX (GPS TX 연결)
#define GPS_TX_PIN 43  // ESP32 TX (GPS RX 연결)
#define GPS_EN_PIN 40  // GPS 켜는 핀

// GPS 속도 (이걸 바꿔가며 테스트해야 함: 9600, 38400, 57600, 115200)
#define GPS_BAUD 9600 

// WiFi 설정 (ESP32가 공유기가 됨)
const char *ssid = "Drone_Tuning";
const char *password = "12345678";

// UDP 설정
WiFiUDP udp;
const int udpPort = 4210;
IPAddress broadcastIP(192, 168, 4, 255); // 브로드캐스트 주소

HardwareSerial GPSSerial(1); // UART1 사용

void setup() {
  // [핵심] Serial.begin()을 절대 하지 않습니다!
  // PC와의 연결을 끊어서 GPS 간섭을 최소화합니다.

  // 1. GPS 전원 켜기
  pinMode(GPS_EN_PIN, OUTPUT);
  digitalWrite(GPS_EN_PIN, HIGH);
  delay(1000);

  // 2. WiFi AP 모드 시작
  WiFi.softAP(ssid, password);
  udp.begin(udpPort);

  // 3. GPS 시리얼 시작
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  udp.beginPacket(broadcastIP, udpPort);
  udp.write('1');
  udp.endPacket();
}

void loop() {
    udp.beginPacket(broadcastIP, udpPort);
    udp.write('1');
    udp.endPacket();
  // GPS에서 데이터가 들어오면?
  if (GPSSerial.available()) {
    // 한 글자 읽어서
    char c = GPSSerial.read();
    
    // 바로 UDP로 쏴버림 (PC로 전송)
    udp.beginPacket(broadcastIP, udpPort);
    udp.write(c);
    udp.write('1');
    udp.endPacket();
  }
}