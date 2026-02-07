#include <ESP32Servo.h> 

// 모터 객체 생성
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

// 핀 번호 (ESP32-S3)
const int pin1 = 4;
const int pin2 = 5;
const int pin3 = 6;
const int pin4 = 7;

// ==========================================
// [설정] 스로틀 범위 지정
// ==========================================
const int MIN_THROTTLE = 1000; // 0%
const int MAX_THROTTLE = 2000; // 100% (참고용)
const int LIMIT_THROTTLE = 1300; // 30% 제한 (1000 + 300)

void setup() {
  // 디버깅용으로 시리얼은 켜두는 게 좋습니다 (필수는 아님)
  Serial.begin(115200);

  // 1. ESC 설정 (50Hz 표준 PWM)
  motor1.setPeriodHertz(50);
  motor2.setPeriodHertz(50);
  motor3.setPeriodHertz(50);
  motor4.setPeriodHertz(50);

  // 2. 핀 연결 (1000~2000us 범위)
  motor1.attach(pin1, MIN_THROTTLE, MAX_THROTTLE);
  motor2.attach(pin2, MIN_THROTTLE, MAX_THROTTLE);
  motor3.attach(pin3, MIN_THROTTLE, MAX_THROTTLE);
  motor4.attach(pin4, MIN_THROTTLE, MAX_THROTTLE);

  // 3. 안전장치: 일단 0으로 시작
  writeAll(MIN_THROTTLE);

  Serial.println("=========================================");
  Serial.println(">>> AUTO TEST MODE (LIMIT 30%) <<<");
  Serial.println("WARNING: Motors will start in 7 seconds!");
  Serial.println("=========================================");

  // [중요] ESC가 켜지고 '아밍(삐-삐-)' 소리를 들을 시간을 줍니다.
  // 배터리 꼽고 바로 돌면 위험하니까 7초 카운트다운 합니다.
  delay(7000); 
}

void loop() {
  Serial.println("🚀 Sequence Started (Max 30%)");
  
  // [1단계] 가속 (Ramp Up): 0% -> 30% (1300us)
  // 천천히 올림 (딜레이 10ms)
  Serial.println(">>> Ramping Up...");
  for (int throttle = MIN_THROTTLE; throttle <= LIMIT_THROTTLE; throttle += 5) {
    writeAll(throttle);
    delay(15); 
  }

  // [2단계] 유지 (Hold): 30% 상태로 2초간 회전
  Serial.println(">>> Holding at 30%...");
  delay(2000); 

  // [3단계] 감속 (Ramp Down): 30% -> 0%
  Serial.println(">>> Ramping Down...");
  for (int throttle = LIMIT_THROTTLE; throttle >= MIN_THROTTLE; throttle -= 5) {
    writeAll(throttle);
    delay(15);
  }

  // [4단계] 휴식 (Rest): 3초간 정지 후 다시 반복
  Serial.println("✅ Stopped. Waiting 3 sec...");
  writeAll(MIN_THROTTLE);
  delay(3000);
}

// 4개 모터 동시 제어 함수
void writeAll(int us) {
  motor1.writeMicroseconds(us);
  motor2.writeMicroseconds(us);
  motor3.writeMicroseconds(us);
  motor4.writeMicroseconds(us);
}