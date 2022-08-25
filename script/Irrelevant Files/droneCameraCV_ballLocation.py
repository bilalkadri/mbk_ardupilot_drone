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


sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
import cv2 
import time
import imutils
from collections import deque


dronetype='/webcam'

bridge = CvBridge()  #Convert Image messages between ROS and OPenCV

empty_msg = Empty()
twist=Twist()
#Float32MultiArray=
colorRange=((0, 240, 240), (20, 255, 255))
pts = deque(maxlen=64)

#global x,y,radius
#global dataToSend
#global ballLocation
def nothing(x):
    pass


def takeoff():
    global pub_TakeOff
    pub_TakeOff.publish(Empty())


def land():
    global pub_land
    pub_land.publish(Empty())


def ImageCallBack(img_msg):
    try:
         cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
         
         
    except CvBridgeError, e:
         rospy.logerr("CvBridge Error: {0}".format(e))

    global colorRange
    global x,y,radius
    global ballLocation  #Why this variable is defined again, ballLocation is a global variable
                         # that has already been defined
    
    height,width, channel = cv_image.shape
    frame=     cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)
    cv_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)
    blurred =  cv2.GaussianBlur(frame, (13, 13), 0)  #Why is he blurring reason was defined in Anees Kouba's notes
    #cv2.imshow("blurred",blurred)
    hsv =      cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    #cv2.imshow("hsv",hsv)
    
    mask = cv2.inRange(hsv, colorRange[0], colorRange[1])
    #cv2.imshow("mask1",mask)

    # Function of erosin is to remove white noise
    #all the pixels near boundary will be discarded depending upon the size of kernel. 
    # So the thickness or size of the foreground object decreases or simply white region 
    # decreases in the image. It is useful for removing small white noises
    mask = cv2.erode(mask, None, iterations=2)
    #cv2.imshow("mask2",mask)
    
    
    # It is just opposite of erosion. Here, a pixel element is 1 if atleast one pixel under
    #  the kernel is 1. So it increases the white region in the image or size of foreground
    #  object increases. Normally, in cases like noise removal, erosion is followed by 
    # dilation. Because, erosion removes white noises, but it also shrinks our object. 
    # So we dilate it. Since noise is gone, they wont come back, but our object area 
    # increases. It is also useful in joining broken parts of an object.
    mask = cv2.dilate(mask, None, iterations=2)
    #cv2.imshow("mask3",mask)
    
	# find contours in the mask and initialize the current
	# (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None

	# only proceed if at least one contour was found
    if len(cnts) > 0:

		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
        
    	c = max(cnts, key=cv2.contourArea)
    	((x, y), radius) = cv2.minEnclosingCircle(c)
        #print x,y,radius
    	M = cv2.moments(c)
    	center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		# only proceed if the radius meets a minimum size
    	if radius > 10:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(cv_image, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(cv_image, center, 5, (0, 0, 0), -1)
    
    
	# update the points queue
    pts.appendleft(center)

	# loop over the set of tracked points
    for i in range(1, len(pts)):
		# if either of the tracked points are None, ignore
		# them
		if pts[i - 1] is None or pts[i] is None:
			continue

		# draw the connecting lines

		thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)
		cv2.line(cv_image, pts[i - 1], pts[i], (255, 0, 0), thickness)


	# show the frame to our screen
    #cv2.imshow("Frame", frame)


    #defining the X and Y setpoint for the controller, the controller tries to maintian the 
    #ardrone such that the ball is in the centre of the image
    setPointX=int(width/2)
    setPointY=int(height/2)
    setpointLenght=25
    
    array=[setPointX,setPointY,setpointLenght,x,y,radius]
    dataToSend=Float32MultiArray(data=array)
    ballLocation.publish(dataToSend)

    diffX=x-setPointX
    diffY=setPointY-y
    x=int(x)
    y=int(y)
    #print diffX,diffY, radius
    
    cv2.line(cv_image,(setPointX,0),(setPointX,height),(0,255,0),2)
    cv2.line(cv_image,(setPointX,y),(x,y),(255,0,0),2)
    cv2.line(cv_image,(0,setPointY),(width,setPointY),(0,255,0),2)
    cv2.line(cv_image,(x,setPointY),(x,y),(255,0,0),2)
    cv2.putText(cv_image,str(diffX),(x,setPointY),cv2.FONT_HERSHEY_PLAIN,1,(255,255,255),2)
    cv2.putText(cv_image,str(diffY),(setPointX,y),cv2.FONT_HERSHEY_PLAIN,1,(255,255,255),2)

    #cv_image = imutils.resize(cv_image, width=150)
    cv_image=cv2.resize(cv_image,(320,180))
    cv2.imshow('show Image',cv_image)
    
    cv2.waitKey(1)

def BottomImageCallBack(img_msg):
    try:
         cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError, e:
         rospy.logerr("CvBridge Error: {0}".format(e))
    cv2.imshow('show Bottom',cv_image)
    cv2.waitKey(1)


if __name__ == '__main__':
    
    rospy.init_node('BallTracking')   #name of the node is BallTracking
    
    
    #global ballLocation
    #BallTracking node will publish /ballLocation topic
    #pub = rospy.Publisher('topic_name', std_msgs.msg.String, queue_size=10)
    ballLocation=rospy.Publisher('/ballLocation',Float32MultiArray,queue_size=1)
    


    #Subscribing to the image from the front camera  of ardrone
    #This is the most important function, whenever there is an image 
    #available ImageCallBack function is called and it provides the location of the RED ball

    imageSub=rospy.Subscriber(dronetype+"/image_raw",Image,ImageCallBack)

    #bottomimageSub=rospy.Subscriber(dronetype+"/bottom/image_raw",Image,BottomImageCallBack)

    #I don't understand the meaning of these two lines 
    #Why he is publishing Empty messages on takeoff and land
    pub_TakeOff = rospy.Publisher(dronetype+"/takeoff", Empty, queue_size=10)
    pub_land = rospy.Publisher(dronetype+"/land", Empty, queue_size=10)
    
    if dronetype=='/ardrone':
        pub_move = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    else:
        pub_move = rospy.Publisher(dronetype+'/cmd_vel', Twist, queue_size=10)

    
    rospy.spin()
    

    '''
    rate = rospy.Rate(10) # 10hz
    count = 0
    print("takeoff")
    takeoff()
    while not rospy.is_shutdown():
        if count<10:
            print("takeoff")
            takeoff()
        elif count<50:
            twist.linear.y=0.2
            pub_move.publish(twist)
            print ("move y=5")
        elif count>50 and count<100:
            twist.linear.y=-0.2
            pub_move.publish(twist)    
            print ("move y=-5")
        elif count>100:
            twist.linear.y=0
            pub_move.publish(twist)
            
            print('y=0')
            land()
            print("Landing")

        count=count+1    
        print(count)
        rate.sleep()



    '''




