#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jun  2 15:46:04 2021

@author: badass
"""

import time
import csv
import datetime
import paho.mqtt.client as mqtt

broker_address = "127.0.0.1"

def on_connect(client, userdata, flags, rc):
    if rc==0:
        connected_flag=True #set flag
        print("Connected OK Returned code=",rc)
        client.subscribe("$SYS/broker/clients/#")
        client.subscribe("command")
        client.subscribe("/measurement/#")
        
def on_disconnect(client, userdata, rc):
    print("Disconnected")
    connected_flag=False #set flag
    controller.loop_stop()
    
def on_subscribe(client, userdata, mid, granted_qos):
    #print("Subscription to topic successful")       
    pass

def on_message(client, userdata, message):
    if message.topic == "command":
        print("Command loopback ok")
    if "measurement" in message.topic:
        print("Received measurement")
        
if __name__=="__main__":
    controller = mqtt.Client("controller",clean_session=True)
    
    controller.on_connect = on_connect
    controller.on_subscribe = on_subscribe
    controller.on_message = on_message

    controller.connect(broker_address)
    controller.loop_start()
    
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
                print("CSV file closed and event loop thread stopped")
                break