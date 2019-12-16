#common imports
from __future__ import print_function
import time
import math
import argparse  

#dronekit imports 
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command

#image processing imports
import cv2
import numpy as np
from keras.models import Model
from keras.models import Sequential
from keras_preprocessing.image import ImageDataGenerator
from keras.optimizers import Adam, RMSprop
from keras.layers import Convolution1D, concatenate, SpatialDropout1D, GlobalMaxPool1D, GlobalAvgPool1D, Embedding, \
    Conv2D, SeparableConv1D, Add, BatchNormalization, Activation, GlobalAveragePooling2D, LeakyReLU, Flatten
from keras.layers import Dense, Input, Dropout, MaxPooling2D, Concatenate, GlobalMaxPooling2D, GlobalAveragePooling2D, \
    Lambda, Multiply, LSTM, Bidirectional, PReLU, MaxPooling1D
from keras.layers.pooling import _GlobalPooling1D
from keras.losses import binary_crossentropy
import keras
from keras.preprocessing import image

#communication imports
from firebase import firebase
# import urllib3, urllib, http.client
# import json

#Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()
    
connection_string = args.connect
sitl = None
count1=0

#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

#intialize the camera 
model = keras.models.load_model('interiit_model_3.model')
cap = cv2.VideoCapture(0)
print("Started Camera")

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
if sitl is not None:
    vehicle = connect(connection_string, wait_ready=True)
else:
    vehicle = connect(connection_string, wait_ready=True, baud=57600)

print('Connected via', connection_string)

# firebase initialisation
firebase = firebase.FirebaseApplication('https://inter-iit-drdo-sase-2019.firebaseio.com/', None)
count1 = 0
obj_detected = False

#parameters for mission
final_height = 2
final_north = 5
final_east = 2
vehicle.parameters['WPNAV_SPEED'] = 50    

# Drone functions

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5



def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint


def download_mission():
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.

def zigzag(aLocation, north, east, height):

    """
    Adds a takeoff command and four waypoint commands to the current mission. 
    The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).

    The function assumes vehicle.commands matches the vehicle mission state 
    (you must have called download at least once in the session and after clearing the mission)
    """	

    cmds = vehicle.commands

    print(" Clear any existing commands")
    cmds.clear() 
    
    print(" Define/add new commands.")
    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
     
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, height))

    #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
    point1 = get_location_metres(aLocation,  north, 0)
    point2 = get_location_metres(point1, 0, east)
    point3 = get_location_metres(point2, -north, 0)
    point4 = get_location_metres(point3, 0, east)
    point5 = get_location_metres(point4,  north, 0)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, height))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point2.lat, point2.lon, height))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point3.lon, height))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, height))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point5.lat, point5.lon, height))
    #add dummy waypoint "5" at point 4 (lets us know when have reached destination)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point5.lat, point5.lon, height))    

    print(" Upload new commands to vehicle")
    cmds.upload()


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

        
#firebase functions
def update_firebase():
    count1=count1+1
    lat=vehicle.location.global_relative_frame.lat 
    lon=vehicle.location.global_relative_frame.lon 
    firebase.put('/drone1/obj'+str(count1),"lat",lat)
    firebase.put('/drone1/obj'+str(count1),"lng",lon)
    firebase.put('/',"count1",count1)   

print('Create a new mission (for current location)')
zigzag(vehicle.location.global_frame,final_north,final_east,final_height)


# From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
print("Starting Guided Mode")
arm_and_takeoff(final_height)
print("Reached target altitude")

print("Starting mission")
# Reset mission set to first (0) waypoint
vehicle.commands.next=0

# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")
print("changed to AUTO Mode")


# Monitor mission. 
# Demonstrates getting and setting the command number 
# Uses distance_to_current_waypoint(), a convenience function for finding the 
#   distance to the next waypoint.

print_timer = time.time()

while True:

    nextwaypoint=vehicle.commands.next

    if((time.time() - print_timer) > 1):
        print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
        print('Height:', vehicle.location.global_relative_frame.alt)
        print_timer = time.time()
  
    #< Image Processing
    ret, frame = cap.read()
    cv2.imshow('frame', frame)
    img=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)

    img=cv2.resize(img,(256,256))
    img=(img.astype(np.float32)/255)
    img=np.expand_dims(img, axis=0)
    predictions=model.predict(img,verbose=0)

    y_pred_binary = (predictions > 0.5).astype(np.int)

    if y_pred_binary:
        print("****************************************")
        print("----------------------------------------")
        print("          Object Detected")
        print("----------------------------------------")
        print("****************************************")
        print("****************************************")
        print("----------------------------------------")
        print("At GPS location: ", vehicle.location.global_relative_frame)
        print("----------------------------------------")
        print("****************************************")

        # firebase update
        update_firebase()
    # />

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    if nextwaypoint==6 : #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
        print("Exit 'standard' mission when start heading to final waypoint (5)")
        break;

vehicle.mode = VehicleMode("LAND")
print("changed to LAND Mode")


#Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

# Close camera
cap.release()
cv2.destroyAllWindows()
