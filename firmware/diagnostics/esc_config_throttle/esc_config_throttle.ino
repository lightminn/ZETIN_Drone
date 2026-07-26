/*
 * esc_config_throttle.ino
 *
 * 목적: EMAX "BLHeli Series" ESC의 TX 스로틀 프로그래밍을 위해
 *       한 채널에 표준 50 Hz RC PWM(1000~2000 us)을 출력한다.
 *       기본 핀 GPIO7은 비행 펌웨어의 M4 / RL(후-좌) 모터다.
 *
 * WARNING: 반드시 프로펠러를 제거(props OFF)하고, 전류 제한 전원공급기를
 * 사용하며, 설정할 ESC/모터 하나만 연결한 상태에서 실행한다.
 * 부팅 시 출력은 항상 1000 us(최저 스로틀)에서 시작한다.
 *
 * ESP32-S3에서는 ESP32Servo가 LEDC 채널을 잘못 할당해 한 GPIO 신호로
 * 두 모터가 돌 수 있었으므로 사용하지 않는다. 이 스케치는 ledcAttach /
 * ledcWrite를 직접 사용하며, LEDC 하드웨어가 현재 설정값을 계속 출력한다.
 */

#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

// 다른 모터를 설정할 때 이 한 줄만 해당 신호 GPIO로 바꾼다.
constexpr uint8_t MOTOR_PIN = 7;  // 기본값: M4 / RL(후-좌)

constexpr uint32_t ESC_FREQ_HZ = 50;
constexpr uint8_t ESC_RES_BITS = 14;  // ESP32-S3 LEDC 최대 해상도
constexpr uint32_t ESC_PERIOD_US = 1000000UL / ESC_FREQ_HZ;
constexpr uint32_t ESC_MAX_DUTY = (1UL << ESC_RES_BITS) - 1UL;

constexpr int MIN_PULSE_US = 1000;
constexpr int MID_PULSE_US = 1500;
constexpr int MAX_PULSE_US = 2000;

constexpr size_t COMMAND_BUFFER_SIZE = 32;

int currentPulseUs = MIN_PULSE_US;
bool escAttached = false;
char commandBuffer[COMMAND_BUFFER_SIZE];
size_t commandLength = 0;

uint32_t pulseUsToDuty(int pulseUs) {
  // 반올림을 적용해 50 Hz / 14-bit에서 가능한 가장 가까운 펄스 폭을 만든다.
  return ((uint64_t)pulseUs * ESC_MAX_DUTY + ESC_PERIOD_US / 2)
         / ESC_PERIOD_US;
}

void writeCurrentPulse() {
  if (escAttached) {
    ledcWrite(MOTOR_PIN, pulseUsToDuty(currentPulseUs));
  }
}

void printCurrentPulse() {
  Serial.print("[PWM] current = ");
  Serial.print(currentPulseUs);
  Serial.println(" us");
}

void setPulseUs(long requestedUs) {
  int clampedUs;
  if (requestedUs < MIN_PULSE_US) {
    clampedUs = MIN_PULSE_US;
  } else if (requestedUs > MAX_PULSE_US) {
    clampedUs = MAX_PULSE_US;
  } else {
    clampedUs = (int)requestedUs;
  }

  if (requestedUs != clampedUs) {
    Serial.print("[CLAMP] requested ");
    Serial.print(requestedUs);
    Serial.print(" us -> ");
    Serial.print(clampedUs);
    Serial.println(" us");
  }

  if (clampedUs != currentPulseUs) {
    currentPulseUs = clampedUs;
    writeCurrentPulse();
  }
  printCurrentPulse();
}

void printCommandMenu() {
  Serial.println();
  Serial.println("[시리얼 명령 — 명령 입력 후 Enter]");
  Serial.println("  h       : 2000 us (full/top)");
  Serial.println("  l       : 1000 us (min/bottom)");
  Serial.println("  m       : 1500 us (mid; Bidirectional neutral only)");
  Serial.println("  1000~2000 정수 : 해당 us로 설정 (범위 밖 값은 clamp)");
  Serial.println("  ? / help: 명령 메뉴와 BLHeli 프로그래밍 절차 표시");
}

void printProgrammingQuickReference() {
  Serial.println();
  Serial.println("[BLHeli TX-PROGRAMMING QUICK REFERENCE — Rotation Direction]");
  Serial.println("  1) Set throttle to top ('h'), THEN power on the ESC.");
  Serial.println("     After ~2 s: \"beep-beep\", then tune \"123 123\" = program mode.");
  Serial.println("  2) Keep at top; 8 parameter indicator tones cycle.");
  Serial.println("     Rotation Direction = one long beep + three short beeps.");
  Serial.println("  3) On that tone, within 2 s set throttle to bottom ('l').");
  Serial.println("     This enters the Rotation Direction menu; values repeat.");
  Serial.println("  4) Value order: Normal, Reverse, Bidirectional.");
  Serial.println("     On the 2nd (Reverse) tone, within 2 s set throttle to top ('h').");
  Serial.println("     Tune \"123 123 1\" = saved.");
  Serial.println("  5) Exit: set bottom ('l') and hold -> exit tune -> normal running.");
  Serial.println("  WARNING: choose Reverse (2nd option), NOT Bidirectional (3rd option).");
  Serial.println("           Bidirectional makes min-throttle mean mid-stick.");
}

void printHelp() {
  printCommandMenu();
  printProgrammingQuickReference();
}

char *trimWhitespace(char *text) {
  while (*text != '\0' && isspace((unsigned char)*text)) {
    ++text;
  }

  char *end = text + strlen(text);
  while (end > text && isspace((unsigned char)*(end - 1))) {
    --end;
  }
  *end = '\0';
  return text;
}

void processCommand(char *rawCommand) {
  char *command = trimWhitespace(rawCommand);
  if (*command == '\0') {
    return;
  }

  if (strcmp(command, "h") == 0) {
    setPulseUs(MAX_PULSE_US);
    return;
  }
  if (strcmp(command, "l") == 0) {
    setPulseUs(MIN_PULSE_US);
    return;
  }
  if (strcmp(command, "m") == 0) {
    Serial.println(
        "[WARNING] 1500 us is neutral only for Bidirectional mode; "
        "do not select Bidirectional when setting M4 Reverse.");
    setPulseUs(MID_PULSE_US);
    return;
  }
  if (strcmp(command, "?") == 0 || strcmp(command, "help") == 0) {
    printHelp();
    return;
  }

  char *parseEnd = nullptr;
  long requestedUs = strtol(command, &parseEnd, 10);
  while (*parseEnd != '\0' && isspace((unsigned char)*parseEnd)) {
    ++parseEnd;
  }
  if (parseEnd != command && *parseEnd == '\0') {
    setPulseUs(requestedUs);
    return;
  }

  Serial.print("[ERROR] unknown command: ");
  Serial.println(command);
  printCommandMenu();
}

void setup() {
  // 시리얼 초기화보다 먼저 최저 스로틀 PWM을 시작한다.
  escAttached = ledcAttach(MOTOR_PIN, ESC_FREQ_HZ, ESC_RES_BITS);
  writeCurrentPulse();

  Serial.begin(115200);
  Serial.println();
  Serial.println("=========================================");
  Serial.println(">>> ESC CONFIG THROTTLE (DIRECT LEDC) <<<");
  Serial.print("Signal pin: GPIO");
  Serial.print(MOTOR_PIN);
  Serial.println(" (default GPIO7 = M4 / RL)");
  Serial.print("LEDC: 50 Hz, 14-bit attach = ");
  Serial.println(escAttached ? "OK" : "FAIL (no PWM output)");
  printCurrentPulse();
  Serial.println(
      "SAFETY: props OFF, PSU current-limited, only this one motor connected.");
  Serial.println("=========================================");
  printHelp();
}

void loop() {
  // LEDC 하드웨어가 별도 갱신 없이 currentPulseUs 신호를 계속 출력한다.
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r' || c == '\n') {
      if (commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        processCommand(commandBuffer);
        commandLength = 0;
      }
      continue;
    }

    if (commandLength < COMMAND_BUFFER_SIZE - 1) {
      commandBuffer[commandLength++] = c;
    } else {
      commandLength = 0;
      Serial.println("[ERROR] command too long; input cleared.");
    }
  }
}
