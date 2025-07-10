#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "../lib/sensor.h"
#include "../lib/connect.h"

Adafruit_MPU6050 mpu;

// Complementary filter coefficient
float COMPLEMENTARY_FILTER_ALPHA = 0.98;

// Attitude angle variables
float angle_roll = 0;
float angle_pitch = 0;
float angle_yaw = 0;

// Time control variables
unsigned long loop_timer;
float dt;

void setup()
{
  Serial.begin(115200);
  // Skipping MPU6500 initialization (for testing without sensor)
  Serial.println("Skipping MPU6500 initialization (sensorless test mode).");

  delay(100);

  // Initialize timer for the first loop's dt calculation
  loop_timer = micros();

  // CSV output header
  Serial.println("Roll,Pitch,Yaw");

  delay(1000);

  // Setup WiFi and TCP Server
  setupWiFi();
  startTCPServer();
}

void loop()
{
  updateSensor();

  handleClientConnection();
  
  // Print results
  Serial.print(angle_roll);
  Serial.print(",");
  Serial.print(angle_pitch);
  Serial.print(",");
  Serial.println(angle_yaw);

  delay(10);
}
