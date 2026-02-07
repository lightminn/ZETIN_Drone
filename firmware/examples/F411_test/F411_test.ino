/*
 * STM32 Nucleo F411RE - "RX Listen" Test
 * 시리얼 모니터에서 아무 키나 입력해서 전송하면 LED가 켜집니다.
 * (TX가 고장 났어도 RX가 살아있는지 확인 가능)
 */

void setup() {
  pinMode(PA5, OUTPUT);
  digitalWrite(PA5, LOW); // 일단 끔

  // Nucleo 국룰 핀 설정
  Serial.setRx(PA3); 
  Serial.setTx(PA2);
  Serial.begin(115200);
  
}

void loop() {
  // 컴퓨터로부터 데이터가 들어오면?
  if (Serial.available() > 1) {
    char c = Serial.read(); // 데이터를 읽어서 버퍼 비움
    Serial.print("1");
    // LED 켰다 끄기 (수신 확인)
    digitalWrite(PA5, HIGH);
    delay(100);
    digitalWrite(PA5, LOW);
    delay(100);
  }
}