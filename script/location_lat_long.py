import asyncio
import json
import websockets
import random
import rospy
from sensor_msgs.msg import NavSatFix

# This file sends the lat long data of the drone on the AWS Cloud
# The lat, long is taken from mavros using a subscriber

global latitude_mavros
global longitude_mavros

# async def send_location():
#     # uri = "ws://13.53.193.224:8000/ws/location/"  #production
#     uri = "ws://localhost:8000/ws/location/" #development
#     async with websockets.connect(uri) as websocket:
#         print("WebSocket connection established")
        
#         latitude = 24.7938
#         longitude = 67.1347
#         data = {
#             "latitude": latitude,
#             "longitude": longitude
#         }
        
#         # Send data to the server
#         await websocket.send(json.dumps(data))
        
#         # Receive message from the server
#         message = await websocket.recv()
#         print(f"Message received from server: {message}")

def posecb(data):
    global latitude_mavros
    global longitude_mavros
    latitude_mavros = data.latitude
    longitude_mavros = data.longitude

async def send_location():
    global latitude_mavros
    global longitude_mavros

    # 44.2.2.54.13 is the public IP of the AWS Cloud
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
            #Subscribe to the MAVROS Global Location
            rospy.Subscriber('/mavros/global_position/global', NavSatFix, posecb)

            # Add 0.005 to the current latitude and longitude
            # latitude += 0.00005
            # longitude += 0.00005

            latitude = latitude_mavros
            longitude = longitude_mavros

            data = {
                "latitude": latitude,
                "longitude": longitude
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