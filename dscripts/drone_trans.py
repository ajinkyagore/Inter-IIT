import RPi.GPIO as GPIO
from time import sleep
import datetime
from firebase import firebase


import urllib2, urllib, httplib
import json
import os 
from functools import partial

GPIO.setmode(GPIO.BCM)
GPIO.cleanup()
GPIO.setwarnings(False)


# Example using a Beaglebone Black with DHT sensor
# connected to pin P8_11.
firebase = firebase.FirebaseApplication('https://inter-iit-drone-2020.firebaseio.com/', None)
str_gps="23.34566,78.45766"

count=0
obj_detected=True  # if object is detected this value goes True
def update_firebase(str_gps,count):
    count_total=firebase.get('/count',None)
    
    lat=float(str_gps.split(',')[0])
    lon=float(str_gps.split(',')[1])
    firebase.put('/drone1/obj'+str(count),"lat",lat)
    firebase.put('/drone1/obj'+str(count),"lng",lon)
    
    count_total+=1
    firebase.put('/',"count",count_total)
    
     
    
    # is_count_full(count_total)
        # this function will check if total count is > 5 -> RTL
    
if obj_detected:
    count += 1
    update_firebase(str_gps,count)
    sleep(5)