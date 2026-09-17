#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>

#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#else
#include "wifi_secrets.example.h"
#endif

// D6=RX(우노 Hold A1), D5=TX(우노 Resume A2). 우노 소프트시리얼 9600.
SoftwareSerial uno(D6, D5);

WiFiClient client;
unsigned long lastTryMs = 0;

static void connectWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }
  Serial.print("wifi ");
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("ip ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("wifi fail");
  }
}

static void connectCenter() {
  if (client.connected()) {
    return;
  }
  if (millis() - lastTryMs < 2000) {
    return;
  }
  lastTryMs = millis();
  Serial.print("center ");
  Serial.print(CENTER_HOST);
  Serial.print(":");
  Serial.println(CENTER_PORT);
  if (client.connect(CENTER_HOST, CENTER_PORT)) {
    client.println("H box");
    Serial.println("center ok");
  } else {
    Serial.println("center fail");
  }
}

void setup() {
  Serial.begin(115200);
  uno.begin(9600);
  delay(200);
  Serial.println("bridge ready");
  connectWifi();
}

void loop() {
  connectWifi();
  connectCenter();

  while (client.connected() && client.available()) {
    int c = client.read();
    if (c < 0) {
      break;
    }
    uno.write((uint8_t)c);
  }

  while (uno.available()) {
    int c = uno.read();
    if (c < 0) {
      break;
    }
    if (client.connected()) {
      client.write((uint8_t)c);
    }
  }
}
