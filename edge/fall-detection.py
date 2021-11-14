
from tb_device_mqtt import TBDeviceMqttClient, TBPublishInfo
import paho.mqtt.client as mqtt  		    #mqtt library
import os
import json
import serial
import time
from datetime import datetime


ACCESS_TOKEN='change this'         #Token of your device
broker="thingsboard.cloud"         #host name
port=1883       #data listening port

MQTT_ADDRESS = '192.168.68.116'
MQTT_USER = 'iot'
MQTT_PASSWORD = 'iot'
MQTT_TOPIC = 'home/+/+'

def on_connect(client, userdata, flags, rc):
    """ The callback for when the client receives a CONNACK response from the server."""
    print('Connected with result code ' + str(rc))
    client.subscribe(MQTT_TOPIC)

def on_message(client, userdata, msg):
    """The callback for when a PUBLISH message is received from the server."""
    #print(msg.topic + ' ' + str(msg.payload))
    print(str(msg.payload.decode("utf-8")))
    ax = str(msg.payload.decode("utf-8")[0:4])
    #ax = int(axx[2:6])
    ay = str(msg.payload.decode("utf-8")[5:9])
    az = str(msg.payload.decode("utf-8")[10:14])
    fall_value = str(msg.payload.decode("utf-8")[15:16])
    #print(ax)
    #return(ax,ay,az,fall_value)
    client1 = TBDeviceMqttClient(broker, ACCESS_TOKEN)
    client1.connect()
    telemetry = {'accel_x': ax,'accel_y': ay, 'accel_z': az, 'fall': fall_value}
    client1.send_telemetry(telemetry).get()
    #time.sleep(5)
    
    
def main():
    mqtt_client = mqtt.Client()
    mqtt_client.username_pw_set(MQTT_USER, MQTT_PASSWORD)
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    #mqtt_client.on_publish = on_publish
    
    mqtt_client.connect(MQTT_ADDRESS, 1883)
    mqtt_client.loop_forever()
    

if __name__ == '__main__':
    #print('MQTT to InfluxDB bridge')
    main()


