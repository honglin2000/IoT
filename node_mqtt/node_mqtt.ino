//https://www.instructables.com/How-to-Use-MQTT-With-the-Raspberry-Pi-and-ESP8266/
//check this for setup raspberry mqtt
#include <WiFi.h>
#include <PubSubClient.h>
#include "Config.h"
#define WIFI_TIMEOUT_MS 20000
//const char* ssid = "AFM";                   // wifi ssid
//const char* password =  "88888888";         // wifi password
const char* mqttServer = "192.168.68.116";    // IP adress Raspberry Pi
//IPAddress mqttServer(address[0], address[1], address[2], address[3]);
const int mqttPort = 1883;
//const char* mqttUser = "username";      // if you don't have MQTT Username, no need input
//const char* mqttPassword = "12345678";  // if you don't have MQTT Password, no need input

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {

  Serial.begin(9600);
  
  connectToWiFi();
//  connectToServer();

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");

    if (client.connect("ESP32Client", MQTT_USER, MQTT_PASSWORD )) {

      Serial.println("connected");

    } else {

      Serial.println("failed with state ");
      Serial.print(client.state());
      delay(2000);

    }
  }

//  client.publish("esp8266", "Hello Raspberry Pi");
//  client.subscribe("esp8266");

}

void callback(char* topic, byte* payload, unsigned int length) {

  Serial.print("Message arrived in topic: ");
  Serial.println(topic);

  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }

  Serial.println();
  Serial.println("-----------------------");

}

void loop() {
    client.publish("esp32", "Hello Raspberry Pi");
    client.subscribe("esp32");
    delay(300);
  client.loop();
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi");
  Serial.println(WIFI_SSID);
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
    Serial.print("IP address is : ");
    Serial.println(WiFi.localIP());
  }
}

void connectToServer() {
  if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
    Serial.println("Connected to server.");
  } else {
    Serial.println("Failed to connect to server.");
  }
}
