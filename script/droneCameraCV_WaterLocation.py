#! /usr/bin/python
#!/usr/bin/env python3
#!/usr/bin/env python2

import sys
import numpy as np
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from sensor_msgs.msg import Image
import cv2 
import time
import imutils
from collections import deque


# sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')


# define range of blue color in HSV
#lower_blue = np.array([110,50,50])
#upper_blue = np.array([130,255,255])

dronetype='/webcam'
RADIUS_RED_THRESHOLD=50
RADIUS_BLUE_THRESHOLD=50

bridge = CvBridge()  #Convert Image messages between ROS and OPenCV

empty_msg = Empty()
twist=Twist()
pts_red = deque(maxlen=64)
pts_blue = deque(maxlen=64)

x_blue=0
y_blue=0
radius_blue=0
x_red=0
y_red=0
radius_red=0
# Water_Reservoir_Location =0
# Water_Discharge_Location =0


def nothing(x):
    pass



def Water_Reservoir_Discharge_Location_Identification_CallBack(img_msg):
    
    global x_blue,y_blue,radius_blue
    global x_red,y_red,radius_red
    global Water_Reservoir_Location 
    global Water_Discharge_Location

    try:
        cv_image_red = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        cv_image_blue = bridge.imgmsg_to_cv2(img_msg, "passthrough")
         
    except CvBridgeError as e:
        print('There is some error')
        #rospy.logerr("CvBridge Error: {0}".format(e))

   
    
    height_red,width_red,channel = cv_image_red.shape
    height_blue,width_blue,channel = cv_image_blue.shape

    #The image coming from ROS which has been converted into OpenCV by using 'bridge.imgmsg_to_cv2' will be 
    #in BGR format, we need to convert it into RGB format therefore we will use cv2.cvtcolor function

    frame_red=     cv2.cvtColor(cv_image_red,cv2.COLOR_BGR2RGB)
    cv_image_red = cv2.cvtColor(cv_image_red,cv2.COLOR_BGR2RGB)
    blurred_red =  cv2.GaussianBlur(frame_red, (13, 13), 0)  

    frame_blue=     cv2.cvtColor(cv_image_blue,cv2.COLOR_BGR2RGB)
    cv_image_blue = cv2.cvtColor(cv_image_blue,cv2.COLOR_BGR2RGB)
    blurred_blue =  cv2.GaussianBlur(frame_blue, (13, 13), 0)  

   
    #blur operation is required to get rid off the noise.
    #cv2.imshow("blurred",blurred)
    
    hsv_image_red =       cv2.cvtColor(blurred_red, cv2.COLOR_BGR2HSV)
    hsv_image_blue =      cv2.cvtColor(blurred_blue, cv2.COLOR_BGR2HSV)
    #cv2.imshow("hsv",hsv)

    #Defining RED mask for the Water Discharge location    
    lower_red = np.array([0,50,50])
    upper_red = np.array([30,255,255])
    mask_red = cv2.inRange(hsv_image_red, lower_red, upper_red)

    #Defining BLUE mask for the Water Reservoir location
    lower_blue = np.array([110,50,50])
    upper_blue = np.array([130,255,255])
    mask_blue = cv2.inRange(hsv_image_blue, lower_blue, upper_blue)

    #mask = cv2.inRange(hsv, colorRange[0], colorRange[1])
    #cv2.imshow("mask",mask)

    # Function of erosin is to remove white noise
    # all the pixels near boundary will be discarded depending upon the size of kernel. 
    # So the thickness or size of the foreground object decreases or simply white region 
    # decreases in the image. It is useful for removing small white noises
    mask_red =  cv2.erode(mask_red,  None, iterations=2)
    mask_blue = cv2.erode(mask_blue, None, iterations=2)
    #cv2.imshow("mask2",mask)
    
    
    # It is just opposite of erosion. Here, a pixel element is 1 if atleast one pixel under
    # the kernel is 1. So it increases the white region in the image or size of foreground
    # object increases. Normally, in cases like noise removal, erosion is followed by 
    # dilation. Because, erosion removes white noises, but it also shrinks our object. 
    # So we dilate it. Since noise is gone, they wont come back, but our object area 
    # increases. It is also useful in joining broken parts of an object.
    mask_red =  cv2.dilate(mask_red,  None, iterations=2)
    mask_blue = cv2.dilate(mask_blue, None, iterations=2)
    #cv2.imshow("mask3",mask)
    
	# find contours in the mask and initialize the current
	# (x, y) center of the Water Reservoir
    cnts_blue = cv2.findContours(mask_blue.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts_blue = imutils.grab_contours(cnts_blue)
    center_blue = None

    
    # (x, y) center of the Water Discharge Location
    cnts_red = cv2.findContours(mask_red.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts_red = imutils.grab_contours(cnts_red)
    center_red = None

    

	# only proceed if at least one contour was found
    if len(cnts_red) > 0:

        
        # Water_Reservoir_Location_Detected: 0
        # Water_Discharge_Location_Detected: 0
        # Water_Sucked_by_Syringes : 0
        # Water_Released_by_Syringes : 0
        
        #Getting the  Water_Released_by_Syringes variable on Prameter_Server
        Water_Released_by_Syringes_local_variable=rospy.get_param("/Water_Released_by_Syringes")
        

        
        
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
        
        c_red = max(cnts_red, key=cv2.contourArea)
        ((x_red, y_red), radius_red) = cv2.minEnclosingCircle(c_red)
        print('RED:Center of min enclosing circle x={0}, y={1}, radius ={2}'.format(x_red,y_red,radius_red))
       
        boundRect_red = cv2.boundingRect(c_red)
        
        #print x,y,radius
        M = cv2.moments(c_red)
        center_red = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # print('Centroid of the Contour=({0},{1})'.format(center_red[0],center_red[1]))
        
		# only proceed if the radius meets a minimum size
        # print('Radius RED is:', radius_red)
        if radius_red > RADIUS_RED_THRESHOLD:
                
                #Setting the parameter on the ROS Parameter Server
                lap_counter = rospy.get_param('/Lap_Count')
        
                if lap_counter==1:
                    rospy.set_param('/Water_Discharge_Location_Detected_Lap_01',1)
                    index=rospy.get_param('/Current_Waypoint_Index_Lap_01')
                    rospy.set_param('/Waypoint_Index_After_which_RED_was_detected',index)


                if (not(Water_Released_by_Syringes_local_variable) and (lap_counter>1)):
                    rospy.set_param('/Water_Discharge_Location_Detected_Lap_02',1)

                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(cv_image_red, (int(x_red), int(y_red)), int(radius_red),(0, 255, 255), 2)
        
                cv2.circle(cv_image_red, center_red, 5, (0, 0, 0), -1)
                #image = cv2.rectangle(image, start_point, end_point, color, thickness)
                cv2.rectangle(cv_image_red, (int(boundRect_red[0]), int(boundRect_red[1])), (int(boundRect_red[0]+boundRect_red[2]), int(boundRect_red[1]+boundRect_red[3])), (0, 0, 0), 2)    
        
        
    
	# only proceed if at least one contour was found
    if len(cnts_blue) > 0:

        
        # Water_Reservoir_Location_Detected: 0
        # Water_Discharge_Location_Detected: 0
        # Water_Sucked_by_Syringes : 0
        # Water_Released_by_Syringes : 0
        
        #Getting the  Water_Sucked_by_Syringes variable from Prameter_Server
        Water_Sucked_by_Syringes_local_variable=rospy.get_param("/Water_Sucked_by_Syringes")
        
        
        # find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and a bounding rectangle
		
        
        c_blue = max(cnts_blue, key=cv2.contourArea)
        ((x_blue, y_blue), radius_blue) = cv2.minEnclosingCircle(c_blue)
        #print x,y,radius
        # print('BLUE: Center of min enclosing circle x={0}, y={1}, radius ={2}'.format(x_blue,y_blue,radius_blue))
        # print('Area of the min enclosing circle A={0}'.format(cv2.contourArea(c_blue)))
        boundRect_blue = cv2.boundingRect(c_blue)
         

        
        M = cv2.moments(c_blue)
        center_blue = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # print('Centroid of the Contour=({0},{1})'.format(center_blue[0],center_blue[1]))
		# only proceed if the radius meets a minimum size
        if radius_blue > RADIUS_BLUE_THRESHOLD:
                
                #Setting the parameter on the ROS Parameter Server
                lap_counter = rospy.get_param('/Lap_Count')

                if lap_counter==1:
                    rospy.set_param('/Water_Reservoir_Location_Detected_Lap_01',1)
                    index=rospy.get_param('/Current_Waypoint_Index_Lap_01')
                    rospy.set_param('/Waypoint_Index_After_which_BLUE_was_detected',index)

                if (not(Water_Sucked_by_Syringes_local_variable) and (lap_counter>1)):
                    rospy.set_param('/Water_Reservoir_Location_Detected_Lap_02',1)

                
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                
                #Drawing the minimum enclosing circle 
                cv2.circle(cv_image_blue, (int(x_blue), int(y_blue)), int(radius_blue),(0, 255, 255), 2)
                
                #
                cv2.circle(cv_image_blue, center_blue, 5, (0, 0, 0), -1)
                cv2.rectangle(cv_image_blue, (int(boundRect_blue[0]), int(boundRect_blue[1])),(int(boundRect_blue[0]+boundRect_blue[2]), int(boundRect_blue[1]+boundRect_blue[3])), (0, 0, 0), 2)    
                
    
	# update the points queue
    pts_red.appendleft(center_red)
    pts_blue.appendleft(center_blue)

	# loop over the set of tracked RED points
    for i in range(1, len(pts_red)):
		# if either of the tracked points are None, ignore
		# them
        if pts_red[i - 1] is None or pts_red[i] is None:
            continue

		# draw the connecting lines

        thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)
        cv2.line(cv_image_red, pts_red[i - 1], pts_red[i], (150, 150,150), thickness)

	# loop over the set of tracked BLUE points
    for i in range(1, len(pts_blue)):
		# if either of the tracked points are None, ignore
		# them
        if pts_blue[i - 1] is None or pts_blue[i] is None:
            continue

		# draw the connecting lines

        thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)
        cv2.line(cv_image_blue, pts_blue[i - 1], pts_blue[i], (150, 150, 150), thickness)




	# show the frame to our screen
    #cv2.imshow("Frame", frame)


    #defining the X and Y setpoint for the controller, the controller tries to maintian the 
    #quadcopter such that the ball comes in the centre of the image
    

    #setPointX and setPointY is the exact location at which the quadcopter is currently hovering
    # i.e. the center of the image acquired from the downward facing camera

    setPointX_red=int(width_red/2)   
    setPointY_red=int(height_red/2)
    
    setPointX_blue=int(width_blue/2)   
    setPointY_blue=int(height_blue/2)
                      
    #'x','y' and 'radius' are the quantities detremined by OpenCV library after processing the image coming
    # from downward camera. These quantities are defined with the origin of (x,y) axis at the top left
    # corner. Hence they are in completely different reference frame as compared to the quadcopter reference frame
    
    array_red =[setPointX_red, setPointY_red, x_red, y_red, radius_red]
    array_blue=[setPointX_blue,setPointY_blue,x_blue,y_blue,radius_blue]
    #Variable definition
    # setPointX = half the size of the image window in x-direction
    # setPointY = half the size of the image window in y-direction
    # setpointZ = height to be maintained above the quadcopter
    # x = position of the target w.r.t the x-axis of the image window (as determined by the minimum enclosing circle using OPenCV library)
    # y = position of the target w.r.t the y-axis of the image window (as determined by the minimum enclosing circle using OPenCV library)
    # radius = radius of the minimum enclosing circle (as determined by the minimum enclosing circle using OPenCV library)
   


    dataToSend_red=Float32MultiArray(data=array_red)
    dataToSend_blue=Float32MultiArray(data=array_blue)
    
    Water_Reservoir_Location.publish(dataToSend_blue)
    Water_Discharge_Location.publish(dataToSend_red)

    #These variable have nothing to do with the Controller or anything, it is just for display purpose
    diffX_blue=x_blue-setPointX_blue  #What is the reason for this variable,no valid reason, it can be setPointX-x as well, it is just to show the text on the image
    diffY_blue=setPointY_blue-y_blue
    x_blue=int(x_blue)
    y_blue=int(y_blue)
    
    diffX_red=x_red-setPointX_red  #What is the reason for this variable,no valid reason, it can be setPointX-x as well, it is just to show the text on the image
    diffY_red=setPointY_red-y_red
    x_red=int(x_red)
    y_red=int(y_red)
    
    #---------------------------------------------------------------------------------------------
    #------------------------------------WATER DISCHARGE LOCATION (RED)----------------------------
    #---------------------------------------------------------------------------------------------

    #Drawing Green coordinate axes i.e. x-axis and y-axis respectively onto the image
    cv2.line(cv_image_red,(setPointX_red,0),(setPointX_red,height_red),(0,255,0),2)
    cv2.line(cv_image_red,(0,setPointY_red),(width_red,setPointY_red),(0,255,0),2)
    
    #Drawing BLACK lines, showing the exact location of the Target,parallel to x and y axis 
    cv2.line(cv_image_red,(setPointX_red,y_red),(x_red,y_red),(0,0,0),2)  #line parallel to x-axis
    cv2.line(cv_image_red,(x_red,setPointY_red),(x_red,y_red),(0,0,0),2)  #line parallel to y-axis 
    
    
    cv2.putText(cv_image_red,str(diffX_red),(x_red,setPointY_red),cv2.FONT_HERSHEY_PLAIN,1,(255,255,255),2)
    cv2.putText(cv_image_red,str(diffY_red),(setPointX_red,y_red),cv2.FONT_HERSHEY_PLAIN,1,(255,255,255),2)

    #---------------------------------------------------------------------------------------------
    #------------------------------------WATER RESERVOIR LOCATION (BLUE)--------------------------
    #---------------------------------------------------------------------------------------------

    #Drawing Green coordinate axes i.e. x-axis and y-axis respectively onto the image
    cv2.line(cv_image_blue,(setPointX_blue,0),(setPointX_blue,height_blue),(0,255,0),2)
    cv2.line(cv_image_blue,(0,setPointY_blue),(width_blue,setPointY_blue),(0,255,0),2)
    
    #Drawing BLACK lines, showing the exact location of the Target,parallel to x and y axis 
    cv2.line(cv_image_blue,(setPointX_blue,y_blue),(x_blue,y_blue),(0,0,0),2)  #line parallel to x-axis
    cv2.line(cv_image_blue,(x_blue,setPointY_blue),(x_blue,y_blue),(0,0,0),2)  #line parallel to y-axis 
    
    
    cv2.putText(cv_image_blue,str(diffX_blue),(x_blue,setPointY_blue),cv2.FONT_HERSHEY_PLAIN,1,(255,255,255),2)
    cv2.putText(cv_image_blue,str(diffY_blue),(setPointX_blue,y_blue),cv2.FONT_HERSHEY_PLAIN,1,(255,255,255),2)



    cv_image_red=cv2.resize(cv_image_red,(320,180))
    cv2.imshow('Water Discharge Location',cv_image_red)
    
    cv_image_blue=cv2.resize(cv_image_blue,(320,180))
    cv2.imshow('Water Reservoir Location',cv_image_blue)
    


    cv2.waitKey(1)




if __name__ == '__main__':
    
    
    global Water_Reservoir_Location
    global Water_Discharge_Location
    
    #name of the node is "Water_Reservoir_Discharge_Location_Identification"
    rospy.init_node('Water_Reservoir_Discharge_Location_Identification')   
      
 

    #publish /Water_Reservoir_Location topic
    Water_Reservoir_Location=rospy.Publisher('/Water_Reservoir_Location_Topic',Float32MultiArray,queue_size=10)
    
    Water_Discharge_Location=rospy.Publisher('/Water_Discharge_Location_Topic',Float32MultiArray,queue_size=10)
    


    #Subscribing to the image from the bottom camera  of Arducopter
    #This is the most important function, whenever there is an image 
    #available ImageCallBack function is called and it provides 
    # the images from the camera


    imageSub=rospy.Subscriber(dronetype+"/image_raw",Image,Water_Reservoir_Discharge_Location_Identification_CallBack)

    #bottomimageSub=rospy.Subscriber(dronetype+"/bottom/image_raw",Image,BottomImageCallBack)

    #I don't understand the meaning of these two lines 
    #Why he is publishing Empty messages on takeoff and land
    #pub_TakeOff = rospy.Publisher(dronetype+"/takeoff", Empty, queue_size=10)
    #pub_land = rospy.Publisher(dronetype+"/land", Empty, queue_size=10)
    
    #if dronetype=='/ardrone':
    #    pub_move = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    #else:
    #    pub_move = rospy.Publisher(dronetype+'/cmd_vel', Twist, queue_size=10)

    
    rospy.spin()
    

