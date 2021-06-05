#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jun  2 15:46:04 2021

@author: Alois Mbutura
"""

import csv
import time
import datetime
import paho.mqtt.client as mqtt

broker_address = "127.0.0.1"

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        connected_flag=True #set flag
        print("Connected OK Returned code=",rc)
        client.subscribe("$SYS/broker/clients/#")
        client.subscribe("command")
        client.subscribe("measurement/#")
    
def on_subscribe(client, userdata, mid, granted_qos):       
    pass

def on_message(client, userdata, message):
    if message.topic == "command":
        print("Measurement command sent!")
    if "measurement" in message.topic:
            mqtt_topic_levels = message.topic.split('/')
            if len(mqtt_topic_levels) != 2:
                print("Improperly formatted measurement topic received")
            else:
                measurement_time = datetime.datetime.now()
                device = mqtt_topic_levels[1]
                value = int(message.payload)
                
                print("Measurement received: "
                     f"Device {device} "
                     f"at {measurement_time}\n")
                
                writer.writerow({'Time': measurement_time,
                                 'Device': device,
                                 'measurement':value})
                
        
if __name__=="__main__":
    controller = mqtt.Client("controller",clean_session=True)
    
    controller.on_connect = on_connect
    controller.on_subscribe = on_subscribe
    controller.on_message = on_message

    controller.connect(broker_address)
    controller.loop_start()
    
    #Open CSV file for writing in overwrite mode
    with open("chamberTest.csv", 'w') as measurement_file:
        print("CSV file opened")
        fieldnames = ['Time', 'Device', 'measurement']
        writer = csv.DictWriter(measurement_file, fieldnames=fieldnames)
        writer.writeheader()
        
        while True:
            try:
                controller.publish("command", "measure")
                time.sleep(5)
            except KeyboardInterrupt:
                controller.loop_stop()
                controller.disconnect()
                measurement_file.close()
                print("CSV file closed and event loop thread stopped")
                break
    print("End of program")