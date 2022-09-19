#! /usr/bin/env python
from __future__ import print_function
from pickle import TRUE
# Import ROS.
import rospy
# Import the API.


from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil
import numpy as np
import imutils
from collections import deque
from std_msgs.msg import Float32MultiArray
import sys
import time
import rospy

takeoff_alt = 5

def dist_between_global_coordinates(aLocation1, aLocation2):
    # This formula has been copied from HEIFU project's repository. Link: https://gitlab.pdmfc.com/drones/ros1/heifu/-/blob/master/heifu_interface/formulas.py
    R = 6371e3; #in meters
    latitude1 = math.radians(aLocation1.lat)
    latitude2 = math.radians(aLocation2.lat)
    latitudevariation = math.radians((aLocation2.lat-aLocation1.lat))
    longitudevariation = math.radians((aLocation2.lon-aLocation1.lon))
    a = math.sin(latitudevariation/2) * math.sin(latitudevariation/2) + math.cos(latitude1) * math.cos(latitude2) * math.sin(longitudevariation/2) * math.sin(longitudevariation/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = R * c #in meters
    return distance

def set_destination(lat, lon, alt, wp_index):

    global vehicle

    print("Moving to Waypoint {0}".format(wp_index))
    
    aLocation = LocationGlobalRelative(lat, lon, float(alt))
    
    # goto_position_target_global_int(aLocation)
    vehicle.simple_goto(aLocation)
    dist_to_wp = dist_between_global_coordinates(vehicle.location.global_frame, aLocation) 
    
    while dist_to_wp > 10:
        # print("Distance to Waypoint {0}: {1}".format(wp_index, dist_to_wp))
        dist_to_wp = dist_between_global_coordinates(vehicle.location.global_frame, aLocation) 
    print("Distance to Waypoint {0}: {1}".format(wp_index, dist_to_wp))
    print("Reached Waypoint {0}".format(wp_index))


    time.sleep(1)



def check_waypoint_reached(lat, lon, alt, wp_index):

    global vehicle

    print("I am in checked waypoint {0} reached function".format(wp_index))
    
    aLocation = LocationGlobalRelative(lat, lon, float(alt))
    
    dist_to_wp = dist_between_global_coordinates(vehicle.location.global_frame, aLocation) 
    
    if dist_to_wp < 10:
        # print("Distance to Waypoint {0}: {1}".format(wp_index, dist_to_wp))
        # dist_to_wp = dist_between_global_coordinates(vehicle.location.global_frame, aLocation) 
        flag=1
    elif dist_to_wp > 10:
        flag=0
    # print("I am in Check Waypoint reached function, reached waypoint :{0}".format(wp_index))
    # print('Value of the flag :',flag)
    
    return flag

    # time.sleep(1)




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


def main():
    # Initializing ROS node.
    rospy.init_node("Waypoint_Navigation", anonymous=True)

 
    # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
    rate = rospy.Rate(0.1)
    global vehicle

    # Connect to the Vehicle
    # connection_string = '/dev/ttyUSB0'
    connection_string = 'udp:127.0.0.1:14550'
    print('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True, baud=921600)
    print("Connection Successfully Established!")    

    # wp1 = LocationGlobalRelative(24.7944609, 67.1353376, 5)
    print("Starting Takeoff")
    arm_and_takeoff(takeoff_alt)

    print("Starting Python Waypoint Navigation")
    # set_destination(-35.3632188, 149.1658468, 5, 1)       #Arguments: latitutde, longitude, relative altitude, waypoint number
    # set_destination(24.7944000, 67.1352048,5,1)
    # vehicle.simple_goto(LocationGlobalRelative(-35.3632188, 149.1658468, 5))
    # time.sleep(2)
    
    #Waypoints for moving the quad in a rectangle , KIET's cricket ground
    # 1) 24.7944000	67.1352048	12.000000
    # 2) 24.79439760	67.13521150	10.000000
    # 3) 24.79442070	67.13542070	10.000000
    # 4) 24.79477870	67.13535640	10.000000
    # 5) 24.79475190	67.13510290	10.000000
    # 6) 24.79439520	67.13516730	10.000000

    # set_destination(24.79439760,67.13521150,5,2)
    # time.sleep(2)
    # set_destination(24.794442070,67.13542070,5,3)
    # time.sleep(2)
    # set_destination(24.79477870,67.13535640,5,4)
    # time.sleep(2)
    # set_destination(24.79475190,67.13510290,5,5)
    # time.sleep(2)
    # set_destination(24.79439520,67.13516730,5,6)
    # time.sleep(2)

    # rospy.set_param('/Lap_Count', 2)


    # set_destination(24.7944000, 67.1352048,5,1)
    # time.sleep(2)
    # set_destination(24.79439760,67.13521150,5,2)
    # time.sleep(2)
    # set_destination(24.794442070,67.13542070,5,3)
    # time.sleep(2)
    # set_destination(24.79477870,67.13535640,5,4)
    # time.sleep(2)
    # set_destination(24.79475190,67.13510290,5,5)
    # time.sleep(2)
    # set_destination(24.79439520,67.13516730,5,6)
    # time.sleep(2)

 

   
    # # Specify GPS waypoints
    goals = [[ 24.7944000, 67.1352048,5,1], 
             [ 24.79439760,67.13521150,5,2], 
             [ 24.794442070,67.13542070,5,3],
             [24.79477870,67.13535640,5,4], 
             [24.79475190,67.13510290,5,5], 
             [ 24.79439520,67.13516730,5,6], 
             [ 24.7944000, 67.1352048,5,7], 
             [ 24.79439760,67.13521150,5,8],
             [24.794442070,67.13542070,5,9], 
             [24.79477870,67.13535640,5,10], 
             [ 24.79475190,67.13510290,5,11],
             [24.79439520,67.13516730,5,12],
             
             ]

    i = 0

    
    while i < len(goals):

        lap_counter = rospy.get_param('/Lap_Count')
     #     #print('Lap Count=',lap_counter)
        Water_Reservoir_Location_Detected_local_variable=rospy.get_param('/Water_Reservoir_Location_Detected')
        Water_Discharge_Location_Detected_local_variable=rospy.get_param('/Water_Discharge_Location_Detected')
        #rospy.loginfo('Water Reservoir',Water_Reservoir_Location_Detected_local_variable)
        
        if i>len(goals)/2:
            rospy.set_param('/Lap_Count', 2)
        
        condition=(not Water_Reservoir_Location_Detected_local_variable) and  (not Water_Discharge_Location_Detected_local_variable) 
     #     #condition= 1 and 1
     #     #print('If condition result',condition)
        rate1 = rospy.Rate(0.1) # ROS Rate at 5Hz
        # if condition:
        #     print('I am executing the PID controller')
        # elif not(condition):
        #     print('I am executing waypoint number :{0}'.format(i))
        if condition:
            x=goals[i][0]
            y=goals[i][1]
            z=goals[i][2] 
            way_point_index=goals[i][3]
            set_destination(x,y,z,way_point_index)
                
            # rate.sleep()
            # while TRUE:
            if check_waypoint_reached(x, y, z, way_point_index):
                i += 1
                    # break
                # elif not(check_waypoint_reached(x, y, z, way_point_index)):

                # rate1.sleep()

    print('Completed all Waypoints! Returning to launch')
    vehicle.mode = VehicleMode("RTL")


    #Close vehicle object before exiting script
    print("Close vehicle object")
    vehicle.close()
    # Land after all waypoints is reached.
    # drone.land()
    # rospy.loginfo(CGREEN2 + "All waypoints reached landing now." + CEND)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()