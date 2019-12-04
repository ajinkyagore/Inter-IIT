from time import sleep
from firebase import firebase


import urllib2, urllib, httplib
import json
#from functools import partial

firebase = firebase.FirebaseApplication('https://inter-iit-drone-2020.firebaseio.com/', None)
str_gps="23.34566,78.45766"

count=0
obj_detected=True
def update_firebase(str_gps,count):
    
    
    lat=float(str_gps.split(',')[0])
    lon=float(str_gps.split(',')[1])
    firebase.put('/drone2/obj'+str(count),"lat",lat)
    firebase.put('/drone2/obj'+str(count),"lng",lon)
    
    
    firebase.put('/',"count2",count)
    
    
while True:
    if obj_detected:
        count += 1
        update_firebase(str_gps,count)
        sleep(1)
    count_total=firebase.get('/count',None)
    # is_count_full(count_total)
        # this function will check if total count is > 5 -> RTL
