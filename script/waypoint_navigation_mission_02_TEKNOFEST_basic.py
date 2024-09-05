#! /usr/bin/env python
# Import ROS.
import rospy
# Import the API.
from iq_gnc.py_gnc_functions import *
# To print colours (optional).
from iq_gnc.PrintColours import *



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
    rospy.init_node("WAypoint_Navigation", anonymous=True)

    # Create an object for the API.
    drone = gnc_api()
    
    # Wait for FCU connection.
    drone.wait4connect()
    
    # Wait for the mode to be switched to GUIDED.
    drone.wait4start()

    # Create local reference frame.
    drone.initialize_local_frame()
    
    # Request takeoff with an altitude of 3m.
    #drone.takeoff(3)
    # We don't need to take off
    
    # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
    rate = rospy.Rate(3)


   
    # Specify some waypoints
    # goals = [[0, 0, 3, 0], 
    #          [40, 0, 3, 0], 
    #          [50, 1.63, 3, 0],
    #          [65, 12, 3, 0], 
    #          [70, 30, 3, 0], 
    #          [68, 40, 3, 0],
    #          [57, 55, 3, 0],
    #          [40, 60, 3, 0],
    #          [0 , 60, 3, 0],
    #          [-4.6, 61.3, 3, 0],
    #          [-7, 65, 3, 0],
    #          [-7.5, 67.5, 3, 0],
    #          [-7.47,67.36, 3, 0],
    #          [-6.5,70.62, 3, 0],
    #          [-3.66,74, 3, 0],
    #          [0, 75, 3, 0],
    #          [3, 74.3, 3, 0],
    #          [5.8, 72, 3, 0],
    #          [6.5, 71, 3, 0],
    #          [6.3, 63.1, 3, 0],
    #          [3.7, 61.19, 3, 0],
    #          [0, 60, 3, 0],
    #          [-40, 60, 3, 0],
    #          [-55.5, 55.83, 3, 0],
    #          [-65, 42.2, 3, 0],
    #          [-70, 30, 3, 0],
    #          [-66, 15.63, 3, 0],
    #          [-59, 6.7, 3, 0],
    #          [-53.17, 3.14, 3, 0],
    #          [-50.14, 2, 3, 0],
    #          [-41, 0, 3, 0],
    #          [0, 0, 3, 0]
             
    #          ]


    # (-7.5,67.5)
    # (-7.47,67.36) NEW
    # (-6.5,70.62) NEW
    # (-3.66,74) NEW
    # (0,75)



    rospy.loginfo("I am in the GUIDED Mode, I am going to sleep")
    rospy.sleep(10)
    rospy.loginfo("I am awake")
    rospy.loginfo("I am shifting back to AUTO Mode")
    drone.set_mode("AUTO")

    # i = 0

    
    # while i < len(goals):
    #     drone.set_destination(
    #         x=goals[i][0], y=goals[i][1], z=goals[i][2], psi=goals[i][3])
    #     rate.sleep()
    #     if drone.check_waypoint_reached():
    #         i += 1
    # # Land after all waypoints is reached.
    # drone.land()
    # rospy.loginfo(CGREEN2 + "All waypoints reached landing now." + CEND)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
