#! /usr/bin/python
#!/usr/bin/env python3
#!/usr/bin/env python2

import sys
import time
import numpy as np
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, TwistStamped
from pid_controller_mbk import pid_controller
#import pygame
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import *
from mavros_msgs.msg import State
import math
import threading
from pid_controller_mbk import pid_controller

#-------------------------------------------------------
#EXPLANATION OF THE pid_controller (Kp,Ki,Kd,limit) parameters
# Kp (Proportional Gain)
# Ki (Integral Gain)
# Kd (Derivative Gain)
# limit (controller saturation limit)


controller_z = pid_controller(0.01, .001, 2, 1) # global z, for copter looking at the shelf it is -x
controller_x = pid_controller(0.01, .001, 2, 1) # global x, for copter looking at the shelf it is y (or -y) 
controller_y = pid_controller(0.01, .001, 2, 1) # global y, for copter looking at the shelf it is z
#controller_yaw = pd_controller(0.1, 0.5, 1.0) # global y, for copter looking at the shelf it is z


#Modifying the code for ARDUPILOT
dronetype='/mavros'


empty_msg = Empty()
twist=TwistStamped()
theta=0
water_detected=0 #flag for water detection
current_pose = PoseStamped()


def pos_sub_callback(pose_sub_data):
    global current_pose
    current_pose = pose_sub_data



def Water_Discharge_Detected_Callback_function(data_recieve):

    
    #print('Water Discharge detected')
    #print data_recieve.data[0]
    setPointX=data_recieve.data[0]  # 320, 640/2 , half the size of x-dimension of window 
    setPointY=data_recieve.data[1]  # 180, 360/2 , half the szie of y-dimension of window
            
    
    currentX=data_recieve.data[2]
    currentY=data_recieve.data[3]
    currentDepth=data_recieve.data[4]  #This the radius of the target as calualted by OPenCV library, by adjusting the 
                                       #radius we can control the height of the quadcopter

    Z_position= current_pose.pose.position.z 
    X_Error= setPointX-currentX       #Remember these errors are caluclated in the image frame of reference where
    Y_Error= setPointY-currentY       #(x,y) is the top left corner
   
    Z_Error=Z_position-2 #Setpoint in z-direction for Quadcopter is 2m

    # print('X Error',X_Error) 
    # print('Y Error',Y_Error) 
    # print('depth Error',depth_Error) 
     
    #Reading parameters from the ROS Parameter Server       
    lap_counter = rospy.get_param("/Lap_Count")
    Water_Discharge_Location_Detected = rospy.get_param("/Water_Discharge_Location_Detected")
    
    
    if (lap_counter >1 and Water_Discharge_Location_Detected):
        #for Water tracking (Old Strategy)
        #Our strategy is, 1) locate the water 
        #2) change Yaw so that Quadcopter points in the direction of water
        #3) when error in Yaw is less that 10 degrees i.e. pi/18 then start moving towards the water
        #4)when Yaw is less than 10 degrees and Euclidean distance i.e. 'r' is also less than 0.01 then move in z-directon

        distance_to_center=math.sqrt(X_Error**2+Y_Error**2)  #X_Error and Y_Error are computed in image reference frame
        yaw_error=math.atan2(Y_Error,X_Error)

        #  -----------------------------------------------------------------
        # Very important transformation between quadcopter and image frame
        #  ----------------------------------------------------------------
    

        #Relationship of image reference frame and quadcopter reference frame
        #(x,y) of the image reference frame is at the top left
        #Quacopter reference frame is not aligned with image reference frame
        
        
        #In the morning of 5th August 2021 following was observed
        #for +ve x movement of quacopter (i.e. if twist.twist.linear.x >0), the ball moves in +ve x direction in the image
        #for +ve y movement of quacopter (i.e. if twist.twist.linear.y >0), the ball moves in -ve y direction in the image

        # twist.twist.linear.x=0.05*controller_x.set_current_error(X_Error)
        # twist.twist.linear.y=0.05*controller_y.set_current_error(-Y_Error)
            


        #In the evening of 5th August 2021 following was observed
        #for +ve x movement of quacopter (i.e. if twist.twist.linear.x >0), the ball moves in -ve x direction in the image
        #for +ve y movement of quacopter (i.e. if twist.twist.linear.y >0), the ball moves in +ve y direction in the image

        # twist.twist.linear.x=0.05*controller_x.set_current_error(-X_Error)
        # twist.twist.linear.y=0.05*controller_y.set_current_error(Y_Error)
        

        #  -------------------------------------
        # All controllers at the same time
        #  -------------------------------------
        Water_Released_by_Syringes_local_variable=rospy.get_param('/Water_Released_by_Syringes')

        if distance_to_center > 10:
            twist.twist.linear.x=0.05*controller_x.set_current_error(-X_Error)
            twist.twist.linear.y=0.05*controller_y.set_current_error(Y_Error)
            twist.twist.linear.z=0 #controller_z.set_current_error(depth_Error)
            print('Cond 1, X error ={0}  Y error ={1} Euclidean distance = {2}'.format(X_Error, Y_Error,distance_to_center))

            twist.twist.angular.x=0
            twist.twist.angular.y=0
            twist.twist.angular.z=0 #controller_yaw.set_current_error(depth_Error)
        
        # Water_Reservoir_Location_Detected: 0
        # Water_Discharge_Location_Detected: 0
        # Water_Sucked_by_Syringes : 0
        # Water_Released_by_Syringes : 0

        elif distance_to_center <10:

            twist.twist.linear.x=0.05*controller_x.set_current_error(-X_Error)
            twist.twist.linear.y=0.05*controller_y.set_current_error(Y_Error)
            if Z_Error>0.1:
                twist.twist.linear.z=-10*controller_z.set_current_error(Z_Error)
            
            elif Z_Error>0 and Z_Error <=0.1 and not(Water_Released_by_Syringes_local_variable):
                time.sleep(5)
                #twist.twist.linear.z=-10*controller_z.set_current_error(0) #hold the altitude
                rospy.set_param("/Water_Released_by_Syringes",1)
                rospy.set_param("/Water_Discharge_Location_Detected",0)
                print('I am discharging the water')
    
                # Water_Reservoir_Location_Detected: 0
                # Water_Discharge_Location_Detected: 0
                # Water_Sucked_by_Syringes : 0
                # Water_Released_by_Syringes : 0
                
            #I have to move back the quadcopter to the previous height
            if (3-Z_position)>0.1 and Water_Released_by_Syringes_local_variable:
                twist.twist.linear.z=10*controller_z.set_current_error(3-Z_position)
                print('I am rising')
                

            print('Cond 2,Z position = {0} Z error ={1}  Euclidean distance = {2}'.format(Z_position,Z_Error, distance_to_center))

          
            twist.twist.angular.x=0
            twist.twist.angular.y=0
            twist.twist.angular.z=0 #controller_yaw.set_current_error(0)
        
        
        pub_move.publish(twist)



def Water_Reservoir_Detected_Callback_function(data_recieve):
    
    
    #print('Water Reservoir detected')
    #print data_recieve.data[0]
    setPointX=data_recieve.data[0]  # 320, 640/2 , half the size of x-dimension of window 
    setPointY=data_recieve.data[1]  # 180, 360/2 , half the szie of y-dimension of window
            
    
    currentX=data_recieve.data[2]
    currentY=data_recieve.data[3]
    currentDepth=data_recieve.data[4]  #This the radius of the target as calualted by OPenCV library, by adjusting the 
                                       #radius we can control the height of the quadcopter

    Z_position= current_pose.pose.position.z 
    X_Error= setPointX-currentX       #Remember these errors are caluclated in the image frame of reference where
    Y_Error= setPointY-currentY       #(x,y) is the top left corner
   
    Z_Error=Z_position-2 #Setpoint in z-direction for Quadcopter is 2m

    # print('X Error',X_Error) 
    # print('Y Error',Y_Error) 
    # print('depth Error',depth_Error) 
     
    #Reading parameters from the ROS Parameter Server       
    lap_counter = rospy.get_param("/Lap_Count")
    Water_Reservoir_Location_Detected = rospy.get_param("/Water_Reservoir_Location_Detected")

    
    if (lap_counter >1 and Water_Reservoir_Location_Detected):
        #for Water tracking (Old Strategy)
        #Our strategy is, 1) locate the water 
        #2) change Yaw so that Quadcopter points in the direction of water
        #3) when error in Yaw is less that 10 degrees i.e. pi/18 then start moving towards the water
        #4)when Yaw is less than 10 degrees and Euclidean distance i.e. 'r' is also less than 0.01 then move in z-directon

        distance_to_center=math.sqrt(X_Error**2+Y_Error**2)  #X_Error and Y_Error are computed in image reference frame
        yaw_error=math.atan2(Y_Error,X_Error)

        #  -----------------------------------------------------------------
        # Very important transformation between quadcopter and image frame
        #  ----------------------------------------------------------------
    

        #Relationship of image reference frame and quadcopter reference frame
        #(x,y) of the image reference frame is at the top left
        #Quacopter reference frame is not aligned with image reference frame
        
        
        #In the morning of 5th August 2021 following was observed
        #for +ve x movement of quacopter (i.e. if twist.twist.linear.x >0), the ball moves in +ve x direction in the image
        #for +ve y movement of quacopter (i.e. if twist.twist.linear.y >0), the ball moves in -ve y direction in the image

        # twist.twist.linear.x=0.05*controller_x.set_current_error(X_Error)
        # twist.twist.linear.y=0.05*controller_y.set_current_error(-Y_Error)
            


        #In the evening of 5th August 2021 following was observed
        #for +ve x movement of quacopter (i.e. if twist.twist.linear.x >0), the ball moves in -ve x direction in the image
        #for +ve y movement of quacopter (i.e. if twist.twist.linear.y >0), the ball moves in +ve y direction in the image

        # twist.twist.linear.x=0.05*controller_x.set_current_error(-X_Error)
        # twist.twist.linear.y=0.05*controller_y.set_current_error(Y_Error)
        

        #  -------------------------------------
        # All controllers at the same time
        #  -------------------------------------
        Water_Sucked_by_Syringes_local_variable=rospy.get_param('Water_Sucked_by_Syringes')

        if distance_to_center > 10:
            twist.twist.linear.x=0.05*controller_x.set_current_error(-X_Error)
            twist.twist.linear.y=0.05*controller_y.set_current_error(Y_Error)
            twist.twist.linear.z=0 #controller_z.set_current_error(depth_Error)
            print('Cond 1, X error ={0}  Y error ={1} Euclidean distance = {2}'.format(X_Error, Y_Error,distance_to_center))

            twist.twist.angular.x=0
            twist.twist.angular.y=0
            twist.twist.angular.z=0 #controller_yaw.set_current_error(depth_Error)
        
        
        elif distance_to_center <10:

            twist.twist.linear.x=0.05*controller_x.set_current_error(-X_Error)
            twist.twist.linear.y=0.05*controller_y.set_current_error(Y_Error)
            if Z_Error>0.1 and not(Water_Sucked_by_Syringes_local_variable):
                twist.twist.linear.z=-10*controller_z.set_current_error(Z_Error)
                print('My Z_Error is greater than 0.1 and my z-position is=',Z_position)
            
            elif Z_Error>0 and Z_Error <=0.1 and not(Water_Sucked_by_Syringes_local_variable):
                time.sleep(5)
                #twist.twist.linear.z=-10*controller_z.set_current_error(0) #hold the altitude
                rospy.set_param("/Water_Sucked_by_Syringes",1)
                rospy.set_param("/Water_Reservoir_Location_Detected",0)
                print('I am sucking the water')

                #I have to move back the quadcopter to the previous height
            if (3-Z_position)>0.1 and Water_Sucked_by_Syringes_local_variable:
                twist.twist.linear.z=10*controller_z.set_current_error(3-Z_position)
                print('I am rising')
                
                         
            
            print('Cond 2,Z position = {0} Z error ={1}  Euclidean distance = {2}'.format(Z_position,Z_Error, distance_to_center))

          
            twist.twist.angular.x=0
            twist.twist.angular.y=0
            twist.twist.angular.z=0 #controller_yaw.set_current_error(0)
        
        
        pub_move.publish(twist)


def timeout():
    print("No message received for 0.1 seconds")
    twist.twist.linear.x=0
    twist.twist.linear.y=0
    twist.twist.linear.z=0

    twist.twist.angular.x=0
    twist.twist.angular.y=0
    twist.twist.angular.z=0
    pub_move.publish(twist)

    # Do something      
         


if __name__ == '__main__':
    
    #global water_detected
        
    rospy.init_node('Water_Tracking_PID_Controller')
        
    reciever=rospy.Subscriber("/Water_Reservoir_Location_Topic",Float32MultiArray,Water_Reservoir_Detected_Callback_function,queue_size=10)
    timer = threading.Timer(0.1,timeout) # If 0.1 seconds elapse and no message received on the topic, call timeout()
    timer.start()    

    reciever=rospy.Subscriber("/Water_Discharge_Location_Topic",Float32MultiArray,Water_Discharge_Detected_Callback_function,queue_size=10)


    local_position_subscribe = rospy.Subscriber(dronetype+'/local_position/pose', PoseStamped, pos_sub_callback)    
    
    #We will takeoff and land using Anis Kouba's Code for generating comand to MavProxy (dronemap_control_using_MAVROS)
    #pub_TakeOff = rospy.Publisher(dronetype+"/takeoff", Empty, queue_size=10)
    #pub_land = rospy.Publisher(dronetype+"/land", Empty, queue_size=10)


    pub_move = rospy.Publisher(dronetype+'/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)     
      
    
    
    rospy.spin()


