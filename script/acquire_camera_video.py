#! /usr/bin/python
# MIT License
# Copyright (c) 2019-2022 JetsonHacks
# See LICENSE for OpenCV license and additional information

# https://docs.opencv.org/3.3.1/d7/d8b/tutorial_py_face_detection.html
# On the Jetson Nano, OpenCV comes preinstalled
# Data files are in /usr/sharc/OpenCV
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

dronetype='/mavros'

# gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
# Defaults to 1920x1080 @ 30fps
# Flip the image by setting the flip_method (most common values: 0 and 2)
# display_width and display_height determine the size of the window on the screen
# Notice that we drop frames if we fall outside the processing time in the appsink element

def gstreamer_pipeline(
    capture_width=1280,
    capture_height=1080,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink drop=True"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


def water_location_detect():
    # pub = rospy.Publisher('chatter', String, queue_size=10)
    pub = rospy.Publisher(dronetype+"/image_raw", Image, queue_size=10)
    bridge=CvBridge()

    rospy.init_node('Publisher for camera image', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    window_title = "Water Reservoir  and Discharge Area Detection"

    video_capture = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
    if video_capture.isOpened():
        try:
            # cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
            while not rospy.is_shutdown():
                ret, frame = video_capture.read()

                # Converting the image from OpenCV format to ROS
                try:
                    ros_image=bridge.cv2_to_imgmsg(frame,"passthrough")
                except CvBridgeError as e:
                    print("There is some error")
                    


                pub.publish(ros_image)
                rate.sleep()

                # if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                #     cv2.imshow(window_title, frame)
                # else:
                #     break
                keyCode = cv2.waitKey(10) & 0xFF
                # Stop the program on the ESC key or 'q'
                if keyCode == 27 or keyCode == ord('q'):
                    break
        finally:
            video_capture.release()
            cv2.destroyAllWindows()
    else:
        print("Unable to open camera")

if __name__ == '__main__':
    try:
        water_location_detect()
    except rospy.ROSInterruptException:
        pass
    
