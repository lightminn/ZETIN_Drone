#include <Arduino.h>

// PCB 핀 설정
#define GPS_RX_PIN 44  // ESP32 RX (GPS TX 연결)
#define GPS_TX_PIN 43  // ESP32 TX (GPS RX 연결)
#define GPS_EN_PIN 40  // 핵심: GPS 전원 켜는 핀

HardwareSerial GPSSerial(1); 

void setup() {
  Serial.begin(115200);
  delay(2000); // PC랑 연결될 시간 주기

  Serial.println("\n🚀 GPS Force Power-Up Sequence Started...");
  
  // 1. 여기서 불을 켭니다 (EN 핀 HIGH)
  pinMode(GPS_EN_PIN, OUTPUT);
  Serial.print("Driving EN Pin (GPIO ");
  Serial.print(GPS_EN_PIN);
  Serial.println(") HIGH...");
  
  digitalWrite(GPS_EN_PIN, HIGH); 
  
  delay(1000); // 켜질 때까지 좀 기다려줌
  Serial.println("✅ GPS Power Should be ON now. (Check Red LED)");

  // 2. 통신 시작 (일단 9600으로 고정)
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

void loop() {
  // GPS -> PC (GPS가 하는 말 출력)
  if (GPSSerial.available()) {
    char c = GPSSerial.read();
    Serial.write(c);
  }

  // PC -> GPS (명령어 치면 전달)
  if (Serial.available()) {
    char c = Serial.read();
    GPSSerial.write(c);
  }
}