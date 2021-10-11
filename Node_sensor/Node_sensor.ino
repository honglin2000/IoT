#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include "Config.h"
#define WIFI_TIMEOUT_MS 20000

IPAddress server(address[0], address[1], address[2], address[3]);
WiFiClient client;

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

//int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
//float ax=0, ay=0,az=0, gx=0,gy=0,gz=0;
boolean fall = false;
boolean trigger1 = false, trigger2 = false, trigger3 = false;
boolean trigCount1 = 0, trigCount2 = 0, trigCount3 = 0; //trigger Count
int angleChange = 0;
//void send_event(const char*event);

void setup() {
  Serial.begin(9600);
  //  pinMode(buzzer, OUTPUT);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip!");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  //mpu.getAccelerometerSensor();
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    mpu_read();
    float Amp = pow(pow(a.acceleration.x, 2) + pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2), 0.5);
    //int Amp = Raw_Amp * 10; // Mulitiplied by 10 bcz values are between 0 to 1
    Serial.println(Amp);
    //double mag = sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2));
    //client.println(mag);

    if (Amp <= 2 && trigger2 == false)
    { //if amplitude breaks lower threshold (0.4g)
      trigger1 = true;
      Serial.println("TRIGGER 1 ACTIVATED");
    }
    if (trigger1 == true)
    {
      trigCount1++;
      if (Amp >= 12)
      { //if AM breaks upper threshold (3g)
        trigger2 = true;
        Serial.println("TRIGGER 2 ACTIVATED");
        trigger1 = false; trigCount1 = 0;
      }
    }
    if (trigger2 == true)
    {
      trigCount2++;
      angleChange = pow(pow(g.gyro.x, 2) + pow(g.gyro.y, 2) + pow(g.gyro.z, 2), 0.5); Serial.println(angleChange);
      if (angleChange >= 30 && angleChange <= 400)
      { //if orientation changes by between 80-100 degrees
        trigger3 = true; trigger2 = false; trigCount2 = 0;
        Serial.println(angleChange);
        Serial.println("TRIGGER 3 ACTIVATED");
      }
    }
    if (trigger3 == true)
    {
      trigCount3++;
      if (trigCount3 >= 10)
      {
        angleChange = pow(pow(g.gyro.x, 2) + pow(g.gyro.y, 2) + pow(g.gyro.z, 2), 0.5);
        //delay(10);
        Serial.println(angleChange);
        if ((angleChange >= 0) && (angleChange <= 10))
        { //if orientation changes remains between 0-10 degrees
          fall = true; trigger3 = false; trigCount3 = 0;
          Serial.println(angleChange);
        }
        else
        { //user regained normal orientation
          trigger3 = false; trigCount3 = 0;
          Serial.println("TRIGGER 3 DEACTIVATED");
        }
      }
    }
    if (fall == true)
    { //in event of a fall detection
      Serial.println("FALL DETECTED");
      //send_event("fall_detect");
      fall = false;
    }

  } else if (WiFi.status() != WL_CONNECTED) {
    // (Re)Connect to WiFi network
    connectToWiFi();
  } else if (!client.connected()) {
    // (Re)Connect to server
    client.stop();
    connectToServer();
    delay(1000);
  }
}

/////////////////////////////////////////////////////////////////////////////

void mpu_read() {
  mpu.getEvent(&a, &g, &temp);
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");
  delay(1000);
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
  if (client.connect(server, 12345)) {
    Serial.println("Connected to server.");
  } else {
    Serial.println("Failed to connect to server.");
  }
}


