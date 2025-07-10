#ifdef CONNECT_H
#define CONNECT_H
#include <iostream>
#include <string>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
//--- ★★★★★ 사용자 설정 부분 ★★★★★ ---
const char *ssid = "zetin";         // 본인의 스마트폰 핫스팟 이름
const char *password = "nitez348*"; // 핫스팟 비밀번호
const int TCP_PORT = 8888;          // 통신할 포트 번호
//--- 설정 끝 ---
WiFiServer server(TCP_PORT); // TCP 서버 객체 생성
void setup();
void start();
void stop();
void restart();
void sendCommand(const std::string &command);
void receiveCommand();
void printStatus();

#endif