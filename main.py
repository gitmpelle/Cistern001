#####################################################################################
# Example program for the ADS1115_mPy module
#
# This program shows how to use the ADS1115 in single shot mode. 
#  
# Further information can be found on (currently only for the Arduino version):
# https://wolles-elektronikkiste.de/ads1115 (German)
# https://wolles-elektronikkiste.de/en/ads1115-a-d-converter-with-amplifier (English)
# 
#####################################################################################
from pichromecast import play_url, create_url
import time
import sys
import ubinascii
from umqtt.simple import MQTTClient
import machine
import random
import ntptime
rtc = machine.RTC()
import gc
gc.collect()
from machine import I2C , Pin, deepsleep
from time import sleep
from ADS1115 import *
import esp
esp.osdebug(None)
import json
import esp32

# Publish MQTT messages after every set timeout
last_publish = time.time()
publish_interval = 5

ADS1115_ADDRESS = 0x48
PwrDwn = Pin(15,Pin.OUT)
PwrDwn.value(1)

i2c = I2C(0,scl=Pin(22), sda=Pin(21), freq=10000)
adc = ADS1115(ADS1115_ADDRESS, i2c=i2c)
adc.setVoltageRange_mV(ADS1115_RANGE_6144)
adc.setCompareChannels(ADS1115_COMP_0_GND)
adc.setMeasureMode(ADS1115_SINGLE)

def getTime():
        timestamp=rtc.datetime()
        timestring="%04d-%02d-%02d %02d:%02d:%02d"%(timestamp[0:3] +  timestamp[4:7])
        return f'{timestring[0:20]}'
    
def readChannel(channel):
    adc.setCompareChannels(channel)
    adc.startSingleMeasurement()
    while adc.isBusy():
        pass
    voltage = adc.getResult_V()
    return voltage

# Complete project details at https://RandomNerdTutorials.com

def sub_cb(topic, msg):
  print((topic, msg))
  if topic == b'notification' and msg == b'received':
    print('ESP received hello message')

def connect_and_subscribe():
  global client_id, mqtt_server, topic_sub, topic_pub, port, mqpassword, mquser
  client = MQTTClient(client_id, mqtt_server,user=mquser,password=mqpassword,port=port,keepalive=0)
  client.set_callback(sub_cb)
  client.connect()
  client.subscribe(topic_sub)
  print('Connected to %s MQTT broker, subscribed to %s topic' % (mqtt_server, topic_sub))
  return client

def restart_and_reconnect():
  print('Failed to connect to MQTT broker. Reconnecting...')
  time.sleep(10)
  machine.reset()

# try:
#   client = connect_and_subscribe()
# except OSError as e:
#   restart_and_reconnect()





def main():
    global client_id, mqtt_server, topic_sub, topic_pub, port, mqpassword, mquser
    client = connect_and_subscribe()
    gc.collect()
    print(f"{getTime()} {gc.threshold(gc.mem_free() // 4 + gc.mem_alloc())}")


    while True:
            gc.collect()
            # Non-blocking wait for message
            client.check_msg()
            global last_publish, ring
            PwrDwn.value(0)
            time.sleep(2)
            
            tf = esp32.raw_temperature()
            tc = (tf-32.0)/1.8
            voltage1 = readChannel(ADS1115_COMP_0_GND)*2
            print("Channel 0: {:<4.2f}".format(voltage1))
            voltage2 = ((readChannel(ADS1115_COMP_1_3))-.679)/.0566 # <+12VDC----SENSOR------169R----?GND
            print("Channel 1: {:<4.4f}".format(voltage2))           # 4mA OFFSET 0.679V  0.0566V/IN.  1 FT = .679
            voltage3 = readChannel(ADS1115_COMP_2_GND)
            print("Channel 2: {:<4.2f}".format(voltage3))
            voltage4 = readChannel(ADS1115_COMP_3_GND)
            print("Channel 3: {:<4.2f}".format(voltage4))
            voltage5 = tc
            print("Channel 4: {:<4.2f}".format(voltage5))            
            print("---------------")

            
            if (time.time() - last_publish) >= publish_interval:
                message = {
                'TIME':getTime(),
                'Voltage1':voltage1,
                'Voltage2':voltage2,
                'Voltage3':voltage3,
                'Voltage4':voltage4,
                'Voltage5':voltage5
                }
                payload = json.dumps(message)
                
                client.publish(topic_pub, payload)
                last_publish = time.time()
            PwrDwn.value(1)    
            time.sleep(1)


if __name__ == "__main__":
    while True:
        try:
            client = connect_and_subscribe()
            main()
        except Exception as e:
            sys.print_exception(e)
            
            print(f"{getTime()} {gc.threshold(gc.mem_free() // 4 + gc.mem_alloc())}")
            reset()
            
#sleep(10)                    #delay of 10 seconds
print('Setting to Deep Sleep Mode')
#deepsleep(10000)     #10000ms sleep time
