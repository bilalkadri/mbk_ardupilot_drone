#!/usr/bin/env python

import threading
import rospy
from std_msgs.msg import String

mutex = threading.Lock()


def msg_callback(msg):
    # reentrang processing
    mutex.acquire(blocking=True)
    # work serial port here, e.g. send msg to serial port
    mutex.release()
    # reentrant processing

def timer_callback(event):
    # reentrant processing
    mutex.acquire(blocking=True)
    # work serial port here, e.g. check for incoming data
    mutex.release()
    # reentrant processing, e.g. publish the data from serial port

def service_callback(req):
    # read out temperature
    mutex.acquire(blocking=True)
    # send temperature over serial port
    mutex.release()

    
if __name__ == '__main__':
    # initialize serial port here
    rospy.init_node('locaton counter publisher node')
    # rospy.Subscriber('/test_in', String, msg_callback)
    # rospy.Service('on_req', Empty, service_callback)
    rospy.Timer(rospy.Duration(0.3), timer_callback)
    rospy.spin()