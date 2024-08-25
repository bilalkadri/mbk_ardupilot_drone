import asyncio
import json
import websockets
import random
import rospy
from sensor_msgs.msg import NavSatFix
import numpy as np
import cv2
import base64
from cv_bridge import CvBridge, CvBridgeError

global latitude_mavros
global longitude_mavros


def img_cb(data):
    try:
        np_arr = np.frombuffer(data.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        frame = cv_image
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 35]  #35 is the compression rate
        result, encoded_img = cv2.imencode('.jpg', frame, encode_param)
        # encoded = cv2.imencode('.png',frame)[1]
        # cv2.imshow("df",frame)
        #print(encimg.shape)
        #data = base64.encodestring(encimg)
        data = base64.b64encode(encoded_img)
        data = data.decode('utf-8')
        #print(data)
        #data = base64.encodebytes(encoded_img)
        # print("len(data)")
    except CvBridgeError as e:
          print(e)
        # data =  u''+base64.encodestring(np_arr)
        #if(debug):
        #print(len(data))



def posecb(data):
    global latitude_mavros
    global longitude_mavros
    latitude_mavros = data.latitude
    longitude_mavros = data.longitude

async def send_location():
    global latitude_mavros
    global longitude_mavros

    # define a video capture object
    vid = cv2.VideoCapture(0)

    uri = "ws://44.202.54.13:8000/ws/location/"  #production
    # uri = "ws://localhost:8000/ws/location/" #development
    
    async with websockets.connect(uri) as websocket:
        print("WebSocket connection established")

        latitude = 24.7938
        longitude = 67.1347

        latitude_mavros = latitude
        longitude_mavros = longitude

        rospy.init_node('position_listener', anonymous=True)

        while True:
            # Capture the video frame
            # by frame
            ret, frame = vid.read()
        
            # Display the resulting frame
            # cv2.imshow('frame', frame)

            IMAGE_SHAPE = frame.shape
            encoded_image = base64.encodestring(frame)
            print(type(encoded_image))

            #Subscribe to the MAVROS Global Location
            rospy.Subscriber('/mavros/global_position/global', NavSatFix, posecb)

            # Add 0.005 to the current latitude and longitude
            # latitude += 0.00005
            # longitude += 0.00005

            latitude = latitude_mavros
            longitude = longitude_mavros

            data = {
                "latitude": latitude,
                "longitude": longitude,
                "image": encoded_image.decode('utf-8'),
                "shape": IMAGE_SHAPE,
            }

            # Send data to the server
            await websocket.send(json.dumps(data))

            # Receive message from the server
            message = await websocket.recv()
            print(f"Message received from server: {message}")

            # Add a delay (e.g., 1 second) before sending the next location
            await asyncio.sleep(1)

if __name__ == '__main__':
   
    # Run the WebSocket client
    asyncio.get_event_loop().run_until_complete(send_location())
    rospy.spin()