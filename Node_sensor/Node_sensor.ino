#include <Arduino.h>
#include <WiFi.h>

#include <Adafruit_MPU6050.h>
#include "Config.h"
#define WIFI_TIMEOUT_MS 20000

IPAddress server(address[0], address[1], address[2], address[3]);
WiFiClient client;

Adafruit_MPU6050 mpu;
Adafruit_Sensor* accel;
sensors_event_t event;

void connectToWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS) {
    Serial.print(".");
    delay(100);
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(" Failed!");
  } else {
    Serial.println(" Connected!");
  }
}

void setup() {
  Serial.begin(115200);

  if (!mpu.begin()) {
    Serial.println("Failed to find Hibiscus Sense MPU6050 chip!");
  }

  accel = mpu.getAccelerometerSensor();

}

void loop() {
  if (WiFi.status() == WL_CONNECTED && client.connected()) {
    accel->getEvent(&event);
    double mag = sqrt(pow(event.acceleration.x, 2) + pow(event.acceleration.y, 2) + pow(event.acceleration.z, 2));
    client.print(mag);
    client.print("|");
    Serial.print("Acceleration X: ");
    Serial.print(event.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(event.acceleration.y);
    Serial.print(", Z: ");
    Serial.print(event.acceleration.z);
    Serial.println(" m/s^2");

  } else if (WiFi.status() != WL_CONNECTED) {
    // (Re)Connect to WiFi network
    connectToWiFi();
  }
}
