
import random
import json
from paho.mqtt import client as mqtt_client
import roslibpy
import time
from statistics import mean
#import serial
from math import sin, cos, radians
#from move_base_msgs.msg import MoveBaseActionResult
#import rospy
#from geometry_msgs.msg import PoseStamped
import sys
#import os
 
broker = 'localhost'
port = 1883
#topic = "silabs/aoa/angle/ble-pd-84FD27EEE4E1/ble-pd-60A423C98DB8"
topic = "silabs/aoa/angle/ble-pd-84FD27EEE4E1/ble-pd-842E1431EEDD"
#topic = "silabs/aoa/angle/ble-pd-84FD27EEE4E1/ble-pd-60A423C98CEB"


# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 100)}'
username = ''
password = ''
i = 0
distance_list = []
angle_list = []
list_loop = 9

def connect_mqtt() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def connect_ros():
    
    client_ros = roslibpy.Ros(host='192.168.0.1', port=9090)
    client_ros.on_ready(lambda: print('Is ROS connected?', client_ros.is_connected))
    client_ros.run()
    return client_ros


def getdistance(distance_list,angle_list,client_ros,client):
    
    print('getdistance dis : ',distance_list)
    print('getdistance angle: ',angle_list)
    distance_mode = mean(distance_list)
    angle_mode = mean(angle_list)
    print('getdistance final : ',distance_mode)
    print('getangel final : ',angle_mode)
    distance_list.clear()
    angle_list.clear()
    x0 = 0
    y0 = 0
    d = distance_mode

    final_x = x0 + d*cos(radians(angle_mode))
    final_y = y0 + d*sin(radians(angle_mode))
    finalxy= 'X :'+str(final_x) +' Y :'+ str(final_y)
    print('Coordinate x and Y : ',final_x,final_y)

   
   # this is for publish to move base .................................................
    pub = roslibpy.Topic(client_ros, '/move_base_simple/goal', 'geometry_msgs/PoseStamped') # sending coordinate to robot for movement
    service = roslibpy.Service(client_ros, '/move_base/clear_costmaps', 'std_srvs/Empty')

    docking_x = 0.85
    docking_y = 0.0
    
    docking_z = -0.33
    docking_w = 0.324
    request = roslibpy.ServiceRequest()
    service.call(request)

    print('Sending message to undock...')
    time.sleep(5)
    print('done out from docking...')
    msg_topic ={'header':{'stamp': 1, 'frame_id': 'map'}, 'pose': {'position': {'x': final_x, 'y':final_y, 'z': 0.0}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.1, 'w': 0.1}}}     
    pub.publish(roslibpy.Message(msg_topic))
    print('Sending message...')
    time.sleep(5)   
    print('done...')
    client_ros.terminate()
    client.disconnect()
    sys.exit()
    
    
    

        
        
def subscribe(client: mqtt_client):
    
    def on_message(client, userdata, msg):
        # talker = roslibpy.Topic(client_ros, '/chatter', 'std_msgs/String')
        received = msg.payload.decode("utf-8")
        json_convert = json.loads(received)
        azimuth = json_convert['azimuth']
        elevation = json_convert['elevation']
        distance = json_convert['distance']
        quality = json_convert['quality']
        sequence = json_convert['sequence']

        print("azimuth ",azimuth)
        print("elevation ",elevation)
        print("Distance ",distance)
        print("quality ",quality)
        print('sequence',sequence)

        print("distance len",len(distance_list))
        print("angle len",len(angle_list))
        if (len(distance_list) < list_loop and len(angle_list) < list_loop ):
            distance_list.append(distance,)
            angle_list.append(azimuth)
        else :
            
            client.disconnect()
            print ('stpp')
            
            
       
        print('----------------------------------------------------------')
        
        
      
    client.subscribe(topic)
    client.on_message = on_message


def run():
    while (True):
        client = connect_mqtt()
        client_ros = connect_ros()
        subscribe(client)
        client.loop_forever()
        getdistance(distance_list,angle_list,client_ros,client)


if __name__ == '__main__':
    run()