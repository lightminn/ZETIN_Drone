#ifdef CONNECT_H
#define CONNECT_H
#include <iostream>
#include <string>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
//--- ★★★★★ 사용자 설정 부분 ★★★★★ ---
extern const char *ssid;
extern const char *password;
extern const int TCP_PORT;
extern WiFiServer server;
void setup();
void start();
void stop();
void restart();
void sendCommand(const std::string &command);
void receiveCommand();
void printStatus();

#endif