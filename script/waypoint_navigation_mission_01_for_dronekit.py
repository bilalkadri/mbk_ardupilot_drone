#! /usr/bin/env python

from __future__ import print_function

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


def set_destination(lat, lon, alt, wp_index):

    global vehicle

    print("Moving to Waypoint {0}".format(wp_index))
    
    aLocation = LocationGlobalRelative(lat, lon, float(alt))
    
    # goto_position_target_global_int(aLocation)
    vehicle.simple_goto(aLocation)
    dist_to_wp = dist_between_global_coordinates(vehicle.location.global_frame, aLocation) 
    
    while dist_to_wp > 1:
        print("Distance to Waypoint {0}: {1}".format(wp_index, dist_to_wp))
        dist_to_wp = dist_between_global_coordinates(vehicle.location.global_frame, aLocation) 
    
    print("Distance to Waypoint {0}: {1}".format(wp_index, dist_to_wp))
    print("Reached Waypoint {0}".format(wp_index))


    time.sleep(1)

def goto_position_target_global_int(aLocation):
    """
    Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.

    For more information see: https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_GLOBAL_INT

    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.

    ------------------Coments by MBK----------------------
    This code runs on the actual UAV, we cannot use this code in Gazeo yet
    ----------------------------------------------------------
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000, # type_mask (only speeds enabled)
        aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
        aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)

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

if __name__ == '__main__':

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
    set_destination(24.7944000, 67.1352048,5,1)
    # vehicle.simple_goto(LocationGlobalRelative(-35.3632188, 149.1658468, 5))
    time.sleep(1)
    
    #Waypoints for moving the quad in a rectangle , KIET's cricket ground
    # 1) 24.7944000	67.1352048	12.000000
    # 2) 24.79439760	67.13521150	10.000000
    # 3) 24.79442070	67.13542070	10.000000
    # 4) 24.79477870	67.13535640	10.000000
    # 5) 24.79475190	67.13510290	10.000000
    # 6) 24.79439520	67.13516730	10.000000

    # waypoints_lap_01=[[ 24.7944000, 67.1352048,5,1], 
            
    #          [ 24.794442070,67.13542070,5,2],
    #          [24.79477870,67.13535640,5,3], 
    #          [24.79475190,67.13510290,5,4], 
    #          [ 24.79439520,67.13516730,5,5] ]
    SLEEP_TIME=10
    set_destination(24.7944000, 67.1352048,5,1)
    time.sleep(SLEEP_TIME)
    set_destination(24.794442070,67.13542070,5,2)
    time.sleep(SLEEP_TIME)
    set_destination(24.79477870,67.13535640,5,3)
    time.sleep(SLEEP_TIME)
    set_destination(24.79475190,67.13510290,5,4)
    time.sleep(SLEEP_TIME)
    set_destination(24.79439520,67.13516730,5,5)
    time.sleep(SLEEP_TIME)

    print('Completed all Waypoints! Returning to launch')
    vehicle.mode = VehicleMode("RTL")


    #Close vehicle object before exiting script
    print("Close vehicle object")
    vehicle.close()