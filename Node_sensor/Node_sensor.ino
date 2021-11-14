//#include <Adafruit_MPU6050.h>
//#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "Config.h"
#define BUZZER_PIN 5 // Buzzer's pin 5
#define WIFI_TIMEOUT_MS 20000
#define ledcAttachPin

//IPAddress server(address[0], address[1], address[2], address[3]);
WiFiClient client;
PubSubClient pubclient(mqtt_server, 1883, client);

//Adafruit_MPU6050 mpu;
//sensors_event_t a, g, temp;

String fall_data;
void send_event(const char *event);
const int MPU_addr = 0x68; // I2C address of the MPU-6050
static int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
boolean fall = false;
boolean trigger1 = false, trigger2 = false, trigger3 = false;  //trigger
byte trigCount1 = 0, trigCount2 = 0, trigCount3 = 0; //trigger Count
int angleChange = 0;

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  ledcAttachPin(13, 0);
  Wire.begin(21, 22, 100000);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

}

/*
  This program first checks if the accelerometer value is higher than the lower threshold,
  if so, then it waits for half a second and checks for higher thresholds.
  If the accelerometer value exceeds the upper threshold,
  then it calculates the change in orientation for the gyroscope values.
  When there is a sudden change in orientation,
  it waits for 10 seconds and checks if the orientation remains the same.
*/
void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    mpu_read();
    //Serial.print(AcX); Serial.print(" ");
    //Serial.print(AcY); Serial.print(" "); Serial.println(AcZ);
    ax = (AcX - 2050) / 16384.00;
    ay = (AcY - 77) / 16384.00;
    az = (AcZ - 1947) / 16384.00;
    gx = (GyX + 270) / 131.07;
    gy = (GyY - 351) / 131.07;
    gz = (GyZ + 136) / 131.07;

    //  ax = AcX / 16384.00;
    //  ay = AcY / 16384.00;
    //  az = AcZ / 16384.00;
    //  gx = GyX / 131.07;
    //  gy = GyY / 131.07;
    //  gz = GyZ / 131.07;
    Serial.print(abs(ax)); Serial.print(" ");
    Serial.print(abs(ay)); Serial.print(" "); Serial.println(abs(az));
    String fall_status = "0";  //no fall, status ok

    String axis_data = String(abs(ax)) + " " + String(abs(ay)) + " " + String(abs(az)) + " " + String(fall_status);

    if (pubclient.publish(status_topic, axis_data.c_str())) {
      //Serial.println("Fall status sent!");
    }
    // Again, client.publish will return a boolean value depending on whether it succeded or not.
    // If the message failed to send, we will try again, as the connection may have broken.
    else {
      Serial.println("Fall Status failed to send. Reconnecting to MQTT Broker and trying again");
      pubclient.connect(clientID, mqtt_username, mqtt_password);
      delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
      //pubclient.publish(status_topic, fall_status);
    }

    float Raw_Amp = pow(pow(ax, 2) + pow(ay, 2) + pow(az, 2), 0.5);
    float Amp = Raw_Amp * 10;  // Mulitiplied by 10 bcz values are between 0 to 1
    Serial.print("Amp normal: "); Serial.println(Amp); Serial.print("\n");
    pubclient.publish(fall_topic, axis_data.c_str());
    if (Amp <= 3 && trigger2 == false) { //if AM breaks lower threshold (0.4g)
      trigger1 = true;
      Serial.println("TRIGGER 1 ACTIVATED");
    }
    if (trigger1 == true) {
      trigCount1++;
      if (Amp >= 12) { //if AM breaks upper threshold (3g)
        Serial.print("Amp-1 : "); Serial.println(Amp);
        trigger2 = true;
        Serial.println("TRIGGER 2 ACTIVATED");
        trigger1 = false; trigCount1 = 0;
      }
    }
    if (trigger2 == true) {
      trigCount2++;
      angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
      //Serial.print("Angle-2: "); Serial.println(angleChange);
      if (angleChange >= 40 && angleChange <= 400) { //if orientation changes by between 80-100 degrees
        Serial.print("Amp-2 : "); Serial.println(Amp);
        trigger3 = true; trigger2 = false; trigCount2 = 0;
        Serial.print("Angle-2: "); Serial.println(angleChange);
        Serial.println("TRIGGER 3 ACTIVATED");
      }
    }
    if (trigger3 == true) {
      trigCount3++;
      if (trigCount3 >= 12) {
        angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
        //delay(10);
        //Serial.print("Angle-3: "); Serial.println(angleChange);
        if ((angleChange >= 0) && (angleChange <= 30)) { //if orientation changes remains between 0-10 degrees
          Serial.print("Amp-3 : "); Serial.println(Amp);
          fall = true; trigger3 = false; trigCount3 = 0;
          Serial.print("Angle-3: "); Serial.println(angleChange);
        }
        else { //user regained normal orientation
          trigger3 = false; trigCount3 = 0;
          Serial.println("TRIGGER 3 DEACTIVATED");
        }
      }
    }
    if (fall == true) { //in event of a fall detection
      Serial.println("FALL DETECTED");
      digitalWrite(BUZZER_PIN, HIGH);
      ledcWriteTone(0, 294);
      fall_status = "1";  //fall detected
      //send_event("falldetection");
      String axis_data = String(abs(ax)) + " " + String(abs(ay)) + " " + String(abs(az)) + " " + String(fall_status);
      if (pubclient.publish(status_topic, axis_data.c_str())) {
        Serial.println("Fall Sent!!!");
      }

      delay(3000);
      digitalWrite(BUZZER_PIN, LOW);
      fall = false;
    }
    if (trigCount2 >= 6) { //allow 0.5s for orientation change
      trigger2 = false; trigCount2 = 0;
      Serial.println("TRIGGER 2 DECACTIVATED");
    }
    if (trigCount1 >= 6) { //allow 0.5s for AM to break upper threshold
      trigger1 = false; trigCount1 = 0;
      Serial.println("TRIGGER 1 DECACTIVATED");
    }
    delay(100);
    ////////////////////////////////////////////////////////////

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
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi ");
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
  if (pubclient.connect(clientID, mqtt_username, mqtt_password)) {
    Serial.println("Connected to MQTT Broker!");
  }
  else {
    Serial.println("Connection to MQTT Broker failed...");
  }
}

void send_event(const char *event) {
  Serial.print("Connecting to ");
  Serial.println(host);
  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  const int httpPort = 80;
  if (!client.connect(host, httpPort)) {
    Serial.println("Connection failed");
    return;
  }
  // We now create a URI for the request
  String url = "/trigger/";
  url += event;
  url += "/with/key/";
  url += privateKey;
  Serial.print("Requesting URL: ");
  Serial.println(url);
  // This will send the request to the server
  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "Connection: close\r\n\r\n");
  while (client.connected())
  {
    if (client.available())
    {
      String line = client.readStringUntil('\r');
      Serial.print(line);
    } else {
      // No data yet, wait a bit
      delay(50);
    };
  }
  Serial.println();
  Serial.println("closing connection");
  client.stop();
}
