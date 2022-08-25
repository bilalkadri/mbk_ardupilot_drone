#! /usr/bin/python
#!/usr/bin/env python3
#!/usr/bin/env python2

import sys
import numpy as np
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from pd_controller import pd_controller
import pygame



controller_z = pd_controller(0.04, 0.1, 1.0) # global z, for copter looking at the shelf it is -x
#controller_z = pd_controller(0.13, 0.0, 0.1) # global z, for copter looking at the shelf it is -x
controller_x = pd_controller(0.04, 0.1, 0.7) # global x, for copter looking at the shelf it is y (or -y) 
#controller_x = pd_controller(0.4, 0.0, 0.1) # global x, for copter looking at the shelf it is y (or -y) 
controller_y = pd_controller(0.01, 0.01, 0.7) # global y, for copter looking at the shelf it is z
#controller_y = pd_controller(1.0, 0.0, 1.0) # global y, for copter looking at the shelf it is z
controller_yaw = pd_controller(0.002, 0.0, 1.0) # global y, for copter looking at the shelf it is z

controll='manual'

#Modifying the code for ARDUPILOT
dronetype='/mavros/setpoint_velocity'


empty_msg = Empty()
twist=TwistStamped()


def displayUpdate():
    #screen.fill((255,255,255))
    #font = pygame.font.Font('freesansbold.ttf', 32) 
    if controll=='manual':
        text = font.render('Activated Manual Mode', True, (255,0,0)) 
        screen.blit(text,(200,0))
    elif controll=='auto':
        text = font.render('Activated Tracking Mode', True, (255,0,0)) 
        screen.blit(text,(200,0))
    text = font.render('Press 1 = Tracking-Mode, Press 2 = Manual-Mode', True, (0,0,0)) 
    screen.blit(text,(10,60))
    text = font.render('Press t = Takeoff, Press l = Land', True, (0,0,0)) 
    screen.blit(text,(150,120))
    text = font.render('w-s = Pitch,  a-d = Roll', True, (0,0,0)) 
    screen.blit(text,(200,180))

    text = font.render('q-e = Yaw,  up-down = Altitude', True, (0,0,0)) 
    screen.blit(text,(180,240))
    
    pygame.display.flip()

def takeoff():
    global pub_TakeOff
    pub_TakeOff.publish(Empty())


def land():
    global pub_land
    pub_land.publish(Empty())


def Calculate(data_recieve):
    #print data_recieve.data[0]
    setPointX=data_recieve.data[0]  # 320, 640/2 , half the size of x-dimension of window 
    setPointY=data_recieve.data[1]  # 180, 360/2 , half the szie of y-dimension of window
    setPointDept=data_recieve.data[2] # radious of the ball must be 25 units in the image, if the dron is near the ball then the radious will be large , the ardrone will move away to reduce the radius to 25
        
    
    currentX=data_recieve.data[3]
    currentY=data_recieve.data[4]
    currentDepth=data_recieve.data[5]

    X_Error=setPointX-currentX
    Y_Error=  setPointY-currentY
    dept_Error= setPointDept-currentDepth+15

    
    
   
    global controll
    for event in pygame.event.get():
        if event.type==pygame.KEYDOWN:
            if event.key==pygame.K_t:
                print('Take_off')
                screen.fill((255,255,255))
                text = font.render('TakeOff', True, (0,255,0)) 
                screen.blit(text,(300,300))
                displayUpdate()
                takeoff()
            elif event.key==pygame.K_l:
                print('land')
                controll='manual'
                screen.fill((255,255,255))
                text = font.render('Landing', True, (0,255,0)) 
                screen.blit(text,(300,300))
                displayUpdate()

                twist.linear.x=0
                twist.linear.y=0
                twist.linear.z=0

                twist.angular.x=0
                twist.angular.y=0
                twist.angular.z=0
                pub_move.publish(twist)
                land()
            elif event.key==pygame.K_1:
                controll='auto'
                print('Activate Auto')
                screen.fill((255,255,255))
                text = font.render('Tracking Activated', True, (0,255,0)) 
                screen.blit(text,(250,300))
                displayUpdate()
                

            elif event.key==pygame.K_2:
                controll='manual'
                twist.linear.x=0
                twist.linear.y=0
                twist.linear.z=0
                twist.angular.x=0
                twist.angular.y=0
                twist.angular.z=0
                print('Activate Manual')
                screen.fill((255,255,255))
                text = font.render('Manual Control Activated', True, (0,255,0)) 
                screen.blit(text,(200,300))
                displayUpdate()

            if event.key==pygame.K_a:
                if controll=='manual':
                    print ('left press')
                    twist.linear.y=0.8
                    screen.fill((255,255,255))
                    text = font.render('Move Left', True, (0,255,0)) 
                    screen.blit(text,(300,300))
                    displayUpdate()
            
            elif event.key==pygame.K_d:
                if controll=='manual':
                    print('right press')
                    twist.linear.y=-0.8
                    screen.fill((255,255,255))
                    text = font.render('Move Right', True, (0,255,0)) 
                    screen.blit(text,(300,300))
                    displayUpdate()
            
            
            if event.key==pygame.K_w:
                if controll=='manual':
                    print('forward press')
                    twist.linear.x=0.8
                    screen.fill((255,255,255))
                    text = font.render('Move Forward', True, (0,255,0)) 
                    screen.blit(text,(300,300))
                    displayUpdate()

            elif event.key==pygame.K_s:
                if controll=='manual':
                    print('back press')
                    twist.linear.x=-0.8
                    screen.fill((255,255,255))
                    text = font.render('Move Back', True, (0,255,0)) 
                    screen.blit(text,(300,300))
                    displayUpdate()

            if event.key==pygame.K_q:
                if controll=='manual':
                    print('Anti-clockwise press')
                    twist.angular.z=1.0
                    screen.fill((255,255,255))
                    text = font.render('Rotate Anti-Clockwise', True, (0,255,0)) 
                    screen.blit(text,(220,300))
                    displayUpdate()

            elif event.key==pygame.K_e:
                if controll=='manual':
                    print('clockwise press')
                    twist.angular.z=-1.
                    screen.fill((255,255,255))
                    text = font.render('Rotate Clockwise', True, (0,255,0)) 
                    screen.blit(text,(250,300))
                    displayUpdate()
                    
            if event.key==pygame.K_UP:
                if controll=='manual':
                    print("Go Up press")
                    twist.linear.z=1.0
                    screen.fill((255,255,255))
                    text = font.render('Move Up', True, (0,255,0)) 
                    screen.blit(text,(300,300))
                    displayUpdate()
            elif event.key==pygame.K_DOWN:
                if controll=='manual':
                    print("Go Down press")
                    twist.linear.z=-1.0
                    screen.fill((255,255,255))
                    text = font.render('Move Down', True, (0,255,0)) 
                    screen.blit(text,(300,300))
                    displayUpdate()

        if event.type==pygame.KEYUP:

            if event.key==pygame.K_a:
                if controll=='manual':
                    print ('left release')
                    twist.linear.y=0.0
            
            elif event.key==pygame.K_d:
                if controll=='manual':
                    print('right release')
                    twist.linear.y=0
            
            
            if event.key==pygame.K_w:
                if controll=='manual':
                    print('forward release')
                    twist.linear.x=0

            elif event.key==pygame.K_s:
                if controll=='manual':
                    print('back release')
                    twist.linear.x=0

            if event.key==pygame.K_q:
                if controll=='manual':
                    print('Anti-clockwise release')
                    twist.angular.z=0

            elif event.key==pygame.K_e:
                if controll=='manual':
                    print('clockwise release')
                    twist.angular.z=0
                    
            if event.key==pygame.K_UP:
                if controll=='manual':
                    print("Go Up release")
                    twist.linear.z=0
            elif event.key==pygame.K_DOWN:
                if controll=='manual':
                    print("Go Down release")
                    twist.linear.z=0
       

        

    #print control
    if controll=='auto':
        twist.twist.linear.x=controller_x.set_current_error(dept_Error)
        twist.twist.linear.y=controller_y.set_current_error(X_Error)
        twist.twist.linear.z=controller_z.set_current_error(Y_Error)

        twist.twist.angular.x=0
        twist.twist.angular.y=0
        twist.twist.angular.z=controller_yaw.set_current_error(X_Error)
        pub_move.publish(twist)
        #print twist.linear.x
    elif controll=='manual':
        pub_move.publish(twist)
        #print(twist)
    
        
         


if __name__ == '__main__':
    pygame.init()
    screen = pygame.display.set_mode((800,400))
    pygame.display.set_caption('Quad Controller')
    screen.fill((255,255,255))
    font = pygame.font.Font('freesansbold.ttf', 32) 
    displayUpdate()
    rospy.init_node('Send_Commands')
    reciever=rospy.Subscriber("/ballLocation",Float32MultiArray,Calculate,queue_size=10)
            #rospy.Subscriber("chatter",      String,            callback function)

    
    #We will takeoff and land using Anis Kouba's Code for generating comand to MavProxy (dronemap_control_using_MAVROS)
    #pub_TakeOff = rospy.Publisher(dronetype+"/takeoff", Empty, queue_size=10)
    #pub_land = rospy.Publisher(dronetype+"/land", Empty, queue_size=10)
    
    if dronetype=='/ardrone':
        pub_move = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    else:
        pub_move = rospy.Publisher(dronetype+'/cmd_vel', TwistStamped, queue_size=10)    

    rospy.spin()
