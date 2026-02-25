#include <Arduino.h>
#include <SPI.h>

// 핀 맵핑
#define SPI_MOSI 11
#define SPI_SCK  12
#define SPI_MISO 13

#define ICM1_CS  10 // 오늘 살려낼 주인공
#define ICM2_CS  9  // 입 막아둘 녀석

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n=======================================");
  Serial.println("🩺 ICM1 (10번 핀) 단독 생존 핑 테스트");
  Serial.println("=======================================");

  // 🚨 1. 두 센서 모두 무조건 HIGH로 묶어서 재움 (버스 충돌 방지)
  pinMode(ICM1_CS, OUTPUT); digitalWrite(ICM1_CS, HIGH);
  pinMode(ICM2_CS, OUTPUT); digitalWrite(ICM2_CS, HIGH); // 2번 센서 아가리 봉인

  // 2. SPI 초기화 (CS 자동 제어 끄기: -1)
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, -1);
  
  // SPI 모드 0, 1MHz 속도 (가장 안전하고 호환성 높은 세팅)
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
}

void loop() {
  // =======================================
  // 1번 센서(CS 10) 찌르기
  // =======================================
  digitalWrite(ICM1_CS, LOW);         // 1번 센서야 내 말 들어라 (깨움)
  
  SPI.transfer(0x75 | 0x80);          // WHO_AM_I(0x75) 레지스터 읽기(0x80) 명령 전송
  uint8_t id1 = SPI.transfer(0x00);   // 대답 수신
  
  digitalWrite(ICM1_CS, HIGH);        // 통신 끝 (다시 재움)

  // 결과 출력
  Serial.print("▶️ ICM1 (CS 10) 응답: 0x");
  if (id1 < 16) Serial.print("0");    // 자릿수 맞추기
  Serial.println(id1, HEX);

  // 0x67이 뜨면 축제 시작
  if (id1 == 0x67) {
    Serial.println("   ✅ 미쳤다! 완벽하게 살아났습니다. (0x67 정상)");
  } else if (id1 == 0x00 || id1 == 0xFF) {
    Serial.println("   💀 아직 통신 불가 (납땜/단선/쇼트 추가 확인 필요)");
  } else {
    Serial.println("   ⚠️ 요상한 값 등장 (SPI 노이즈 또는 속도/모드 문제)");
  }

  Serial.println("---------------------------------------");
  delay(1000); // 1초마다 확인
}