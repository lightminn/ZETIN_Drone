#include <Arduino.h>
#include <DShotRMT.h>

// Define the GPIO pin connected to the motor ESC
const gpio_num_t MOTOR_PIN = GPIO_NUM_4;

// Create a DShotRMT instance for DSHOT300
DShotRMT motor(MOTOR_PIN, DSHOT600);

void setup() {
  Serial.begin(115200);

  // Initialize the DShot motor
  motor.begin();

  // Print CPU Info
  printCpuInfo(Serial);

  Serial.println("Motor initialized. Ramping up to 25% throttle...");
  Serial.println("Sending 0% throttle for 2 seconds to arm ESC...");
  motor.sendThrottlePercent(0); // 0% 스로틀 신호 전송
  delay(2000); // ESC가 Arming 될 때까지 2초 대기
  }

void loop() {
  // // Ramp up to 25% throttle over 2.5 seconds
  // for (int i = 0; i <= 50; i++) {
  //   motor.sendThrottlePercent(i);
  //   Serial.println("a");
  //   delay(200);
  // }
  // delay(1000);
  
  // Serial.println("Stopping motor.");
  motor.sendThrottlePercent(1);
  // Serial.println("a");


  // // Print DShot Info
  // printDShotInfo(motor, Serial);

  // // Take a break before next bench run
  // delay(100);
}