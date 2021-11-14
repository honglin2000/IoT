// IoT based Fall Detection using NodeMCU and MPU6050 Sensor
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

// MQTT
const char* mqtt_server = "192.168.68.116";  // IP of the MQTT broker
const char* status_topic = "home/fall/status";
const char* fall_topic = "home/fall/data";
const char* mqtt_username = "iot"; // MQTT username
const char* mqtt_password = "iot"; // MQTT password
const char* clientID = "client_fall"; // MQTT client ID

String fall_data;
const int BUZZER_PIN = 5; // GIOP21 pin connected to Buzzer's pin
const int address[4] = { 192, 168, 68, 116 };
IPAddress server(address[0], address[1], address[2], address[3]);
Adafruit_MPU6050 mpu;
WiFiClient client;
PubSubClient pubclient(mqtt_server, 1883, client);

const int MPU_addr = 0x68; // I2C address of the MPU-6050
static int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
boolean fall = false; //stores if a fall has occurredboolean fall = false; //stores if a fall has occurred
boolean trigger1 = false; //stores if first trigger (lower threshold) has occurred
boolean trigger2 = false; //stores if second trigger (upper threshold) has occurred
boolean trigger3 = false; //stores if third trigger (orientation change) has occurred
byte trigger1count = 0; //stores the counts past since trigger 1 was set true
byte trigger2count = 0; //stores the counts past since trigger 2 was set true
byte trigger3count = 0; //stores the counts past since trigger 3 was set true

int angleChange = 0;

// WiFi network info.
const char *ssid = "change this"; // Enter your Wi-Fi Name
const char *pass = "change this"; // Enter your Wi-Fi Password


void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  Wire.begin(21, 22, 100000);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.println("Wrote to IMU");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  connect_mqtt();
}

void loop() {

  mpu_read();
  ax = (AcX - 2050) / 16384.00;
  ay = (AcY - 77) / 16384.00;
  az = (AcZ - 1947) / 16384.00;
  gx = (GyX + 270) / 131.07;
  gy = (GyY - 351) / 131.07;
  gz = (GyZ + 136) / 131.07;

  Serial.print(ax); Serial.print(" ");
  Serial.print(ay); Serial.print(" "); Serial.println(az);
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
    trigger1count++;
    if (Amp >= 12) { //if AM breaks upper threshold (3g)
      Serial.print("Amp-1 : "); Serial.println(Amp);
      trigger2 = true;
      Serial.println("TRIGGER 2 ACTIVATED");
      trigger1 = false; trigger1count = 0;
    }
  }
  if (trigger2 == true) {
    trigger2count++;
    angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
    //Serial.print("Angle-2: "); Serial.println(angleChange);
    if (angleChange >= 40 && angleChange <= 400) { //if orientation changes by between 80-100 degrees
      Serial.print("Amp-2 : "); Serial.println(Amp);
      trigger3 = true; trigger2 = false; trigger2count = 0;
      Serial.print("Angle-2: "); Serial.println(angleChange);
      Serial.println("TRIGGER 3 ACTIVATED");
    }
  }
  if (trigger3 == true) {
    trigger3count++;
    if (trigger3count >= 12) {
      angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
      //delay(10);
      //Serial.print("Angle-3: "); Serial.println(angleChange);
      if ((angleChange >= 0) && (angleChange <= 30)) { //if orientation changes remains between 0-10 degrees
        Serial.print("Amp-3 : "); Serial.println(Amp);
        fall = true; trigger3 = false; trigger3count = 0;
        Serial.print("Angle-3: "); Serial.println(angleChange);
      }
      else { //user regained normal orientation
        trigger3 = false; trigger3count = 0;
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
  if (trigger2count >= 6) { //allow 0.5s for orientation change
    trigger2 = false; trigger2count = 0;
    Serial.println("TRIGGER 2 DECACTIVATED");
  }
  if (trigger1count >= 6) { //allow 0.5s for AM to break upper threshold
    trigger1 = false; trigger1count = 0;
    Serial.println("TRIGGER 1 DECACTIVATED");
  }
  delay(100);


}

void connect_mqtt() {
  Serial.println("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");              // print ... till not connected
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("Connecting to MQTT");
  // Connect to MQTT Broker
  // client.connect returns a boolean value to let us know if the connection was successful.
  // If the connection is failing, make sure you are using the correct MQTT Username and Password (Setup Earlier in the Instructable)
  if (pubclient.connect(clientID, mqtt_username, mqtt_password)) {
    Serial.println("Connected to MQTT Broker!");
  }
  else {
    Serial.println("Connection to MQTT Broker failed...");
  }
}


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
