#ifndef CONNECT_H
#define CONNECT_H

#include <WiFi.h>

// Initializes WiFi connection
void setupWiFi();

// Starts the TCP server
void startTCPServer();

// Handles incoming client connections and communication
void handleClientConnection();

#endif // CONNECT_H