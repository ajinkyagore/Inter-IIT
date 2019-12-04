from time import sleep
from firebase import firebase


import urllib2, urllib, httplib
import json

firebase = firebase.FirebaseApplication('https://inter-iit-drone-2020.firebaseio.com/', None)
str_gps="23.34566,78.45766"

count1=0
obj_detected=True  # if object is detected this value goes True
def update_firebase(str_gps,count):
    
    
    lat=float(str_gps.split(',')[0])
    lon=float(str_gps.split(',')[1])
    firebase.put('/drone1/obj'+str(count1),"lat",lat)
    firebase.put('/drone1/obj'+str(count1),"lng",lon)
    firebase.put('/',"count1",count1)   
    
while True:
    if obj_detected:
        count1 += 1
        update_firebase(str_gps,count1)#only if object detected
    count_total = firebase.get('/count',None)
    sleep(1)
    # is_count_full(count_total)
        # this function will check if total count is > 5 -> RTL
    
