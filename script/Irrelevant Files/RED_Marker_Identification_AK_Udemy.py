#!/usr/bin/env python

import numpy as np
import cv2
import time

def read_rgb_image(image_name, show):
    rgb_image = cv2.imread(image_name)
    if show: 
        cv2.imshow("RGB Image",rgb_image)
    return rgb_image

def filter_color(rgb_image, lower_bound_color, upper_bound_color):
    #convert the image into the HSV color space
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv image",hsv_image)

   #-----MBK added code-----------------------------------
   # lower mask (0-10)
   #https://stackoverflow.com/questions/30331944/finding-red-color-in-image-using-python-opencv
    lower_red = np.array([0,50,50])
    upper_red = np.array([30,255,255])
    mask = cv2.inRange(hsv_image, lower_red, upper_red)

    # upper mask (170-180)
    #lower_red = np.array([179,50,50])
    #upper_red = np.array([180,255,255])
    #mask1 = cv2.inRange(hsv_image, lower_red, upper_red)


    # join my masks
    #mask = mask0+mask1 
    #----------------------------------------
    
    
    #mask = cv2.inRange(hsv_image, lower_bound_color, upper_bound_color)

    return mask


    

def getContours(binary_image):     
    #_, contours, hierarchy = cv2.findContours(binary_image, 
    #                                          cv2.RETR_TREE, 
    #                                           cv2.CHAIN_APPROX_SIMPLE)
    _, contours, hierarchy = cv2.findContours(binary_image.copy(), 
                                            cv2.RETR_EXTERNAL,
	                                        cv2.CHAIN_APPROX_SIMPLE)

   
    return contours


def draw_ball_contour(binary_image, rgb_image, contours):
    black_image = np.zeros([binary_image.shape[0], binary_image.shape[1],3],'uint8')
    
    for c in contours:
        area = cv2.contourArea(c)
        perimeter= cv2.arcLength(c, True)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        if (area>3000):
            cv2.drawContours(rgb_image, [c], -1, (150,250,150), 1)
            cv2.drawContours(black_image, [c], -1, (150,250,150), 1)
            cx, cy = get_contour_center(c)
            cv2.circle(rgb_image, (cx,cy),(int)(radius),(0,0,255),1)
            cv2.circle(black_image, (cx,cy),(int)(radius),(0,0,255),1)
            cv2.circle(black_image, (cx,cy),5,(150,150,255),-1)
            print ("Area: {}, Perimeter: {}".format(area, perimeter))
    print ("number of contours: {}".format(len(contours)))
    cv2.imshow("RGB Image Contours",rgb_image)
    cv2.imshow("Black Image Contours",black_image)

def get_contour_center(contour):
    M = cv2.moments(contour)
    cx=-1
    cy=-1
    if (M['m00']!=0):
        cx= int(M['m10']/M['m00'])
        cy= int(M['m01']/M['m00'])
    return cx, cy

def detect_ball_in_a_frame(image_frame):

    #define a mask using the lower and upper bounds of the yellow color 
    #yellowLower =(30, 100, 50)
    #yellowUpper = (60, 255, 255)

    redLower =(0, 240, 240)
    redUpper = (60, 255, 255)

    #colorRange=((0, 240, 240), (20, 255, 255))

    rgb_image = image_frame
    #binary_image_mask = filter_color(rgb_image, yellowLower, yellowUpper)
    binary_image_mask = filter_color(rgb_image, redLower, redUpper)


    contours = getContours(binary_image_mask)
    draw_ball_contour(binary_image_mask, rgb_image,contours)



def main():
    #video_capture = cv2.VideoCapture(0)
    #video_capture = cv2.VideoCapture('tcp://192.168.1.3:5555')
    #video_capture = cv2.VideoCapture('video/tennis-ball-video.mp4')
    
    #For ARDRONE 
    video_capture = cv2.VideoCapture()

    video_capture.open('tcp://192.168.1.3:5555')


    while(True):
        ret, frame = video_capture.read()
        detect_ball_in_a_frame(frame)
        time.sleep(0.033)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()



cv2.waitKey(0)
cv2.destroyAllWindows()