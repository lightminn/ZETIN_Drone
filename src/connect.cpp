#include "../lib/connect.h"
#include <Arduino.h>
#include <WiFi.h>

//--- ★★★★★ User-configurable settings ★★★★★ ---
const char *ssid = "zetin";         // Your smartphone hotspot name
const char *password = "nitez348*"; // Hotspot password
const int TCP_PORT = 8888;          // Port number for communication
//--- End of settings ---

WiFiServer server(TCP_PORT); // Create TCP server object

void setupWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" CONNECTED!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void startTCPServer()
{
  server.begin();
  Serial.printf("TCP Server started on port %d\n", TCP_PORT);
}

void handleClientConnection()
{
  WiFiClient client = server.available();

  if (client)
  {
    Serial.println("New client connected!");
    while (client.connected())
    {
      if (client.available())
      {
        String line = client.readStringUntil('\n');
        Serial.print("Received command: ");
        Serial.println(line);

        client.printf("OK, Command Received: %s\n", line.c_str());
        break; 
      }
    }
    client.stop();
    Serial.println("Client disconnected.");
  }
}



