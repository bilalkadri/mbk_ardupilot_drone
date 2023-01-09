#! /usr/bin/env python
#! /usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
import time
import rospy
import logging
import threading


def lap_counter_subscriber_callback_func(msg):
    global lap_counter
    if msg.data<2:
        print('The lap_counter is less than 2')
    elif msg.data==2:
        print('The lap_counter is equal to 2')
    else:
        print('The lap_counter is greater than 2')
  

def lap_counter_publisher_thread_func(name):
    global lap_counter
    logging.info("Thread %s: starting", name)
    print("Thread %s: starting", name)
    lap_count_publisher=rospy.Publisher('lap_counter_topic', Int32,queue_size=10)
    
   
    rate = rospy.Rate(10)  # 10hz
    # lap_count_publisher.publish(lap_counter)
    while not rospy.is_shutdown():
        lap_count_publisher.publish(lap_counter)
        rate.sleep()

if __name__ == '__main__':
    # Initializing ROS node.
    global lap_counter
    lap_counter=0
   
    rospy.init_node("Publisher in a thread", anonymous=True)

    lap_counter_publisher_thread = threading.Thread(target=lap_counter_publisher_thread_func, args=(1,))
    # logging.info("Main    : before running thread")
    lap_counter_publisher_thread.start()

    # lap_counter_subscriber_thread = threading.Thread(target=lap_counter_subscriber_thread_func, args=(1,))
    # # logging.info("Main    : before running thread")
    # lap_counter_subscriber_thread.start()
    lap_counter_subscriber=rospy.Subscriber('lap_counter_topic',Int32,lap_counter_subscriber_callback_func)



    for x in range(5):
        time.sleep(10)
        lap_counter+=1

