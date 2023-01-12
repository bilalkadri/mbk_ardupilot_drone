#! /usr/bin/python
#!/usr/bin/env python3
#!/usr/bin/env python2

from array import array
from pickle import TRUE
import sys
import time
import numpy as np
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, TwistStamped
from pid_controller_mbk import pid_controller
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import *
from mavros_msgs.msg import State
import math
import threading
from pid_controller_mbk import pid_controller
import matplotlib.pyplot as plt
from std_msgs.msg import Int32
import logging


#-------------------------------------------------------
#EXPLANATION OF THE pid_controller (Kp,Ki,Kd,limit) parameters
# Kp (Proportional Gain)
# Ki (Integral Gain)
# Kd (Derivative Gain)
# limit (controller saturation limit)

THRESHOLD_FOR_DISTANCE_TO_CENTER=150
HEIGHT_TO_BE_MAINTAINED_ABOVE_THE_TANK= 3.5

controller_z = pid_controller(0.01, .01, 2, 1) # global z, for copter looking at the shelf it is -x
controller_x = pid_controller(0.01, .01, 2, 1) # global x, for copter looking at the shelf it is y (or -y) 
controller_y = pid_controller(0.01, .01, 2, 1) # global y, for copter looking at the shelf it is z
#controller_yaw = pd_controller(0.1, 0.5, 1.0) # global y, for copter looking at the shelf it is z


#Modifying the code for ARDUPILOT
dronetype='/mavros'


# global variables
empty_msg = Empty()
twist=TwistStamped()
theta=0
water_detected=0 #flag for water detection
current_pose = PoseStamped()

#subscriber call back function for GPS Position
def GPS_Position_Callback_function(data_recieve):
    global   lap_counter
    global   Water_Discharge_Location_Latitude
    global   Water_Discharge_Location_Longitude
    global   Water_Discharge_Location_Altitude
    global   Water_Reservoir_Location_Latitude
    global   Water_Reservoir_Location_Longitude
    global   Water_Reservoir_Location_Altitude
    global   Water_Discharge_Location_Detected_Lap_01 
    global   Water_Reservoir_Location_Detected_Lap_01 
    global   Water_Discharge_Location_Saved
    global   Water_Reservoir_Location_Saved

    latitude=data_recieve.latitude
    longitude=data_recieve.longitude
    altitude=data_recieve.altitude

    #Reading parameters from the ROS Parameter Server       
    #lap_counter = rospy.get_param("/Lap_Count")
    # Water_Discharge_Location_Detected_Lap_01 = rospy.get_param("/Water_Discharge_Location_Detected_Lap_01")
    # Water_Reservoir_Location_Detected_Lap_01 = rospy.get_param("/Water_Reservoir_Location_Detected_Lap_01")

   
       #Saving the GPS location of the Water_Discharge_Location to ROS parameter server 
    if (lap_counter ==1 and Water_Discharge_Location_Detected_Lap_01):
     
        # rospy.set_param('/Water_Discharge_Location_Latitude',latitude)
        # rospy.set_param('/Water_Discharge_Location_Longitude', longitude)
        # rospy.set_param('/Water_Discharge_Location_Altitude' ,altitude)
        Water_Discharge_Location_Latitude=latitude
        Water_Discharge_Location_Longitude=longitude
        Water_Discharge_Location_Altitude=altitude


        # rospy.set_param('/Water_Discharge_Location_Saved',1)
        Water_Discharge_Location_Saved=1
        # rospy.set_param('/Water_Discharge_Location_Detected_Lap_01',0) #so that it does not enter into this if condition again
        #####################################################################################
        ########## ALERT, In this code we are not publishing this variable###################
        ########### We have ro define a separate thread which will publish these variables###
        #####################################################################################
        Water_Discharge_Location_Detected_Lap_01=0 # I think we have to publish this value as well

      #Saving the GPS location of the Water_Reservoir_Location to ROS parameter server 
    if (lap_counter ==1 and Water_Reservoir_Location_Detected_Lap_01):
        # rospy.set_param('/Water_Reservoir_Location_Latitude', latitude)
        # rospy.set_param('/Water_Reservoir_Location_Longitude' ,longitude)
        # rospy.set_param('/Water_Reservoir_Location_Altitude ',altitude)
        Water_Reservoir_Location_Latitude=latitude
        Water_Reservoir_Location_Longitude=longitude
        Water_Reservoir_Location_Altitude=altitude

        # rospy.set_param('/Water_Reservoir_Location_Saved',1)
        Water_Reservoir_Location_Saved=1
        #####################################################################################
        ########## ALERT, In this code we are not publishing this variable###################
        ########### We have ro define a separate thread which will publish these variables###
        #####################################################################################
        # rospy.set_param('/Water_Reservoir_Location_Detected_Lap_01',0) #so that it does not enter into this if condition again
        Water_Reservoir_Location_Detected_Lap_01=0

def Water_Discharge_Detected_Callback_function(data_recieve):
    
    global lap_counter
    # lap_counter=0
    #RED
    
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
   
    Z_Error=Z_position-HEIGHT_TO_BE_MAINTAINED_ABOVE_THE_TANK #Setpoint in z-direction for Quadcopter above the tank

    # print('X Error',X_Error) 
    # print('Y Error',Y_Error) 
    # print('depth Error',depth_Error) 
     
    #Reading parameters from the ROS Parameter Server       
    #lap_counter = rospy.get_param("/Lap_Count")
    Water_Discharge_Location_Detected_Lap_02 = rospy.get_param("/Water_Discharge_Location_Detected_Lap_02")
    EXECUTING_WAYPOINT_NAVIGATION=rospy.get_param('/EXECUTING_WAYPOINT_NAVIGATION') #THis is used so that vehicle.simplegoto() is not interrupted during the execution

  
    if (lap_counter >1 and Water_Discharge_Location_Detected_Lap_02 and not(EXECUTING_WAYPOINT_NAVIGATION)):
        #for Water tracking (Old Strategy)
        #Our strategy is, 1) locate the water 
        #2) change Yaw so that Quadcopter points in the direction of water
        #3) when error in Yaw is less that 10 degrees i.e. pi/18 then start moving towards the water
        #4)when Yaw is less than 10 degrees and Euclidean distance i.e. 'r' is also less than 0.01 then move in z-directon

        # distance_to_center=math.sqrt(X_Error**2+Y_Error**2)  #X_Error and Y_Error are computed in image reference frame
        distance_to_center=math.sqrt(math.pow(X_Error,2)+math.pow(Y_Error,2))  #X_Error and Y_Error are computed in image reference frame
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
        
        #After TEKNOFEST I was making my code compatible with dronekit library
        # This code worked perfrectly with iq_gnc library but with dronekit I was facing a very
        # strange problem. Later on I found out that that dronekit has nothing to do with my 
        # problem. The drone was completing the first lap of Mission # 2, in the second lap it was
        # identifying the blue , descending on it, rising again but when it reaches the red area
        # somehow the PID controllers don't force the quadcopter to center align with the red 
        # area. I was confused for two days on 7th and 8th September 2022. The same PID controller
        # was working fine with blue area. I was totally confused. On the night of 8th SEptember
        # i reversed the signs of X_Error and Y_Error and suddenly it worked, to my utter surprise.
        # Initially I was using -X_Error and Y_Error and I changed it to X_Error and -Y_Error. 
        # The same problem I faced on 5th August 2021. But now I seem to get to the bottom of the problem
        # Actually the angle of approach of the drone towards the blue/red area i.e. Yaw angle at 
        # which the drone enters the blue/red region matters a lot. The sign of X_Error and Y_Error
        # depends on the approach angle which should be corrected. 

        #  -------------------------------------
        # All controllers at the same time
        #  -------------------------------------
        Water_Released_by_Syringes_local_variable=rospy.get_param('Water_Released_by_Syringes')


        if distance_to_center > THRESHOLD_FOR_DISTANCE_TO_CENTER:
            twist.twist.linear.x=0.1*controller_x.set_current_error(X_Error)
            twist.twist.linear.y=0.1*controller_y.set_current_error(-Y_Error)
            twist.twist.linear.z=0 #controller_z.set_current_error(depth_Error)
            # print('RED: Cond 1, X error ={0}  Y error ={1} Euclidean distance = {2}'.format(X_Error, Y_Error,distance_to_center))

            twist.twist.angular.x=0
            twist.twist.angular.y=0
            twist.twist.angular.z=0 #controller_yaw.set_current_error(depth_Error)
        
         # Water_Reservoir_Location_Detected: 0
         # Water_Discharge_Location_Detected: 0
         # Water_Sucked_by_Syringes : 0
         # Water_Released_by_Syringes : 0

        elif distance_to_center <THRESHOLD_FOR_DISTANCE_TO_CENTER:

            twist.twist.linear.x=0.1*controller_x.set_current_error(-X_Error)
            twist.twist.linear.y=0.1*controller_y.set_current_error(Y_Error)
            if Z_Error>1 and not(Water_Released_by_Syringes_local_variable):
                twist.twist.linear.z=-5*controller_z.set_current_error(Z_Error)
            
            elif Z_Error>0 and Z_Error <=1 and not(Water_Released_by_Syringes_local_variable):
                time.sleep(5)
                # twist.twist.linear.z=-10*controller_z.set_current_error(0) #hold the altitude
                rospy.set_param("/Water_Released_by_Syringes",1)
                # rospy.set_param("/Water_Discharge_Location_Detected",0)
                print('I am discharging the water')
    
                # Water_Reservoir_Location_Detected: 0
                # Water_Discharge_Location_Detected: 0
                # Water_Sucked_by_Syringes : 0
                # Water_Released_by_Syringes : 0
                
            #I have to move back the quadcopter to the previous height
            if (5-Z_position)>1 and Water_Released_by_Syringes_local_variable:
                twist.twist.linear.z=5*controller_z.set_current_error(5-Z_position)
                print('I am rising my Z_position is :',Z_position)

            #Adding a new condition here , the quadcopter was stuck after releasing the water
            # since   /Water_Discharge_Location_Detected was set to '0' hence it was not entering
            #in this if condition again
            elif (5-Z_position)<1 and Water_Released_by_Syringes_local_variable:
                rospy.set_param("/Water_Discharge_Location_Detected_Lap_02",0)
                print('I am continuing my mission on the waypoints')   

            # print('Cond 2,Z position = {0} Z error ={1}  Euclidean distance = {2}'.format(Z_position,Z_Error, distance_to_center))

          
            twist.twist.angular.x=0
            twist.twist.angular.y=0
            twist.twist.angular.z=0 #controller_yaw.set_current_error(0)
        
        
        pub_move.publish(twist)



def Water_Reservoir_Detected_Callback_function(data_recieve):
    global lap_counter
    global counter_for_BLUE
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
   
    Z_Error=Z_position-HEIGHT_TO_BE_MAINTAINED_ABOVE_THE_TANK #Setpoint in z-direction for Quadcopter 

    # BLUE_Data=[setPointX,currentX,X_Error,setPointY,currentY,Y_Error,Z_Error]



    # dataTosend_blue=Float32MultiArray(data=BLUE_Data)
    # pub_BLUE_Data.publish(dataTosend_blue)



    # print('X Error',X_Error) 
    # print('Y Error',Y_Error) 
    # print('depth Error',depth_Error) 
     
    #Reading parameters from the ROS Parameter Server       
    # lap_counter = rospy.get_param("/Lap_Count")
    Water_Reservoir_Location_Detected_Lap_02 = rospy.get_param("/Water_Reservoir_Location_Detected_Lap_02")

    EXECUTING_WAYPOINT_NAVIGATION=rospy.get_param('/EXECUTING_WAYPOINT_NAVIGATION') #THis is used so that vehicle.simplegoto() is not interrupted during the execution
    

  
    if (lap_counter >1 and Water_Reservoir_Location_Detected_Lap_02 and not(EXECUTING_WAYPOINT_NAVIGATION)):


        #For plotting currentX, currentY, setPointX, setPointY, X_Error, Y_Error
        # we need to save these values in a list , at the end of the execution we should 
        # save the list in a file
        # counter_for_BLUE=counter_for_BLUE+1
        # currentX_BLUE_Vector.append(currentX)
        # currentY_BLUE_Vector.append(currentY)
        # setPointX_BLUE_Vector.append(setPointX)
        # setPointY_BLUE_Vector.append(setPointY)
        # X_Error_BLUE_Vector.append(X_Error)
        # Y_Error_BLUE_Vector.append(Y_Error)
       
        # index_BLUE_Vector.append(counter_for_BLUE)
        
      
 

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
        # print('distance to center',distance_to_center)
        if distance_to_center > THRESHOLD_FOR_DISTANCE_TO_CENTER:
            twist.twist.linear.x=0.1*controller_x.set_current_error(-X_Error)
            twist.twist.linear.y=0.1*controller_y.set_current_error(Y_Error)
            twist.twist.linear.z=0 #controller_z.set_current_error(depth_Error)
            # print('BLUE:Cond 1, X error ={0}  Y error ={1} Euclidean distance = {2}'.format(X_Error, Y_Error,distance_to_center))

            twist.twist.angular.x=0
            twist.twist.angular.y=0
            twist.twist.angular.z=0 #controller_yaw.set_current_error(depth_Error)
        
        
        elif distance_to_center <THRESHOLD_FOR_DISTANCE_TO_CENTER:

            twist.twist.linear.x=0.1*controller_x.set_current_error(-X_Error)
            twist.twist.linear.y=0.1*controller_y.set_current_error(Y_Error)
            # print('BLUE:Cond 1, X error ={0}  Y error ={1} Euclidean distance = {2}'.format(X_Error, Y_Error,distance_to_center))

            if Z_Error>1 and not(Water_Sucked_by_Syringes_local_variable):
                twist.twist.linear.z=-5*controller_z.set_current_error(Z_Error)
                # print('My Z_Error is greater than 0.1 and my z-position is=',Z_position)
            
            elif Z_Error>0 and Z_Error <=1 and not(Water_Sucked_by_Syringes_local_variable):
                time.sleep(5)
                #twist.twist.linear.z=-10*controller_z.set_current_error(0) #hold the altitude
                rospy.set_param("/Water_Sucked_by_Syringes",1)
                # rospy.set_param("/Water_Reservoir_Location_Detected",0)
                print('I am sucking the water')

                #I have to move back the quadcopter to the previous height
            if (5-Z_position)>1 and Water_Sucked_by_Syringes_local_variable:
                twist.twist.linear.z=5*controller_z.set_current_error(5-Z_position)
                print('I am rising, my Z_position is :',Z_position)
            

            #Adding a new condition here , the quadcopter was stuck after sucking the water
            # since   /Water_Reservoir_Location_Detected was set to '0' hence it was not enter
            #in this if condition again
            elif (5-Z_position)<1 and Water_Sucked_by_Syringes_local_variable:
                rospy.set_param("/Water_Reservoir_Location_Detected_Lap_02",0)
                print('I am continuing my mission on the waypoints')
                
                         
            
           # print('Cond 2,Z position = {0} Z error ={1}  Euclidean distance = {2}'.format(Z_position,Z_Error, distance_to_center))

          
            twist.twist.angular.x=0
            twist.twist.angular.y=0
            twist.twist.angular.z=0 #controller_yaw.set_current_error(0)
        
        # plt.rcParams["figure.figsize"] = [7.50, 3.50]
        # plt.rcParams["figure.autolayout"] = True

        
        # plt.title("Line graph")
        # plt.plot(index_BLUE_Vector, currentX_BLUE_Vector, color="red")

        # plt.show()

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
         
#subscriber callback for WDL_WRL_Parameters subscriber
def WDL_WRL_Parameters_callback(msg):
    global   Water_Discharge_Location_Detected_Lap_01 
    global   Water_Reservoir_Location_Detected_Lap_01 
    global   Water_Discharge_Location_Detected_Lap_02
    global   Water_Reservoir_Location_Detected_Lap_02 
    global   Waypoint_Index_After_which_BLUE_was_detected
    global   Waypoint_Index_After_which_RED_was_detected 
 


    Water_Discharge_Location_Detected_Lap_01    =msg.data[0]
    Water_Reservoir_Location_Detected_Lap_01    =msg.data[1]
    Water_Discharge_Location_Detected_Lap_02    =msg.data[2]
    Water_Reservoir_Location_Detected_Lap_02    =msg.data[3]
    Waypoint_Index_After_which_BLUE_was_detected=msg.data[4]
    Waypoint_Index_After_which_RED_was_detected =msg.data[5]



def pos_sub_callback(pose_sub_data):
    global current_pose
    current_pose = pose_sub_data


#callback for lap_counter_subscriber
def lap_counter_callback(msg):
    global lap_counter
    lap_counter=msg.data

def WDL_WRL_Location_Saved_publisher_thread_func(name):
  global   Water_Discharge_Location_Saved
  global   Water_Reservoir_Location_Saved

  WDL_WRL_Location_Saved_publisher=rospy.Publisher('WDL_WRL_Location_Saved_topic', Float32MultiArray,queue_size=10)
     
  rate = rospy.Rate(10)  # 10hz
   
  while not rospy.is_shutdown():
        array =[Water_Discharge_Location_Saved,
                Water_Reservoir_Location_Saved ]
        dataToSend=Float32MultiArray(data=array)
        WDL_WRL_Location_Saved_publisher.publish(dataToSend)
  
        rate.sleep()


def GPS_publisher_thread_func(name):

    #Defining two publishers in this thread which will keep on updating the GPS coordinates
    # of the Water_Discharge_Location and Water_Reservoir_Location
    # global   lap_counter
    global   Water_Discharge_Location_Latitude
    global   Water_Discharge_Location_Longitude
    global   Water_Discharge_Location_Altitude
    global   Water_Reservoir_Location_Latitude
    global   Water_Reservoir_Location_Longitude
    global   Water_Reservoir_Location_Altitude

    
    logging.info("Thread %s: starting", name)

    Water_Discharge_Location_GPS_publisher=rospy.Publisher('Water_Discharge_Location_GPS_topic', Float32MultiArray,queue_size=10)
     
    Water_Reservoir_Location_GPS_publisher=rospy.Publisher('Water_Reservoir_Location_GPS_topic', Float32MultiArray,queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    # lap_count_publisher.publish(lap_counter)
    while not rospy.is_shutdown():
        WDL_array =[Water_Discharge_Location_Latitude,Water_Discharge_Location_Longitude,Water_Discharge_Location_Altitude ]
        WDL_dataToSend=Float32MultiArray(data=WDL_array)
        Water_Discharge_Location_GPS_publisher.publish(WDL_dataToSend)

        WRL_array =[Water_Reservoir_Location_Latitude,Water_Reservoir_Location_Longitude,Water_Reservoir_Location_Altitude ]
        WRL_dataToSend=Float32MultiArray(data=WRL_array)
        Water_Reservoir_Location_GPS_publisher.publish(WRL_dataToSend)
        rate.sleep()

if __name__ == '__main__':
    
    global   Water_Discharge_Location_Latitude
    global   Water_Discharge_Location_Longitude
    global   Water_Discharge_Location_Altitude
    global   Water_Reservoir_Location_Latitude
    global   Water_Reservoir_Location_Longitude
    global   Water_Reservoir_Location_Altitude
    global   Water_Discharge_Location_Saved
    global   Water_Reservoir_Location_Saved

    Water_Discharge_Location_Latitude=0
    Water_Discharge_Location_Longitude=0
    Water_Discharge_Location_Altitude=0
    Water_Reservoir_Location_Latitude=0
    Water_Reservoir_Location_Longitude=0
    Water_Reservoir_Location_Altitude=0
    Water_Discharge_Location_Saved=0
    Water_Reservoir_Location_Saved=0
    #global water_detected
        
    rospy.init_node('Water_Tracking_PID_Controller')
        
    reciever=rospy.Subscriber("/Water_Reservoir_Location_Topic",Float32MultiArray,Water_Reservoir_Detected_Callback_function,queue_size=10)
   
    reciever=rospy.Subscriber("/Water_Discharge_Location_Topic",Float32MultiArray,Water_Discharge_Detected_Callback_function,queue_size=10)

    reciever=rospy.Subscriber("/mavros/global_position/global",NavSatFix,GPS_Position_Callback_function,queue_size=10)

    local_position_subscribe = rospy.Subscriber(dronetype+'/local_position/pose', PoseStamped, pos_sub_callback)    
    

    #Publishing the GPS locations of WRL and WDL using a separate thread
    GPS_publisher_thread = threading.Thread(target=GPS_publisher_thread_func, args=(1,))
    GPS_publisher_thread.start()

    #Publishing WDL_WRL_Location_Saved using a separate thread
    WDL_WRL_Location_Saved_thread=threading.Thread(target=WDL_WRL_Location_Saved_publisher_thread_func,args(2,))
    WDL_WRL_Location_Saved_thread.start()

    #We will takeoff and land using Anis Kouba's Code for generating comand to MavProxy (dronemap_control_using_MAVROS)
    #pub_TakeOff = rospy.Publisher(dronetype+"/takeoff", Empty, queue_size=10)
    #pub_land = rospy.Publisher(dronetype+"/land", Empty, queue_size=10)


    pub_move = rospy.Publisher(dronetype+'/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)     
  
    #WE would like to save the data that will be used in controllers  
    pub_BLUE_Data=rospy.Publisher('/Water_Reservoir_Data_for_Controller', Float32MultiArray, queue_size=10)
    
    
    #defining subscriber for lap counter
    lap_counter_subscriber=rospy.Subscriber('lap_counter_topic',Int32,lap_counter_callback)

    #defining subscriber for WRL, WDL Parameters
    WRL_WDL_Parameters_subscriber=rospy.Subscriber('WDL_WRL_Parameters_topic',Float32MultiArray,WDL_WRL_Parameters_callback)

    rospy.spin()


