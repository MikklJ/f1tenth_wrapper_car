#!/usr/bin/env python
import gyminterface
from gyminterface import GymInterface
from PIL import Image
import json
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import Twist
import math
import numpy as np
from threading import Thread

GYM_DICT={
    'car':{
    'car_name': 'TritonRacer',
    'font_size': 50,
    'racer_name': 'Triton AI',
    'bio': 'Something',
    'country': 'US',
    'body_style': 'car01',
    'body_rgb': [24, 43, 73],
    'guid': 'some_random_string'},

  'default_connection': 'local', # Which is the default connection profile? "local" or "remote"?
  # default_connection: 'remote'

  'local_connection':{
    'scene_name': 'warren', # roboracingleague_1 | generated_track | generated_road | warehouse | sparkfun_avc | waveshare
    'host': '127.0.0.1', # Use "127.0.0.1" for simulator running on local host.
    'port': 9091,
    'artificial_latency': 0}, # Ping the remote simulator whose latency you would like to match with, and put the ping in millisecond here.

  'remote_connection':{
    'scene_name': 'warren',
    'host': '192.168.0.23', # Use the actual host name for remote simulator.
    'port': 9091,
    'artificial_latency': 0}, # Besides the ping to the remote simulator, how many MORE delay would you like to add?

  'lidar':{
    'enabled': False,
    'deg_inc': 2, # Degree increment between each ray of the lidar
    'max_range': 50.0}, # Max range of the lidar laser
}
    
class Wrapper:
    def __init__(self):
        self.gym = GymInterface(gym_config = GYM_DICT)
        self.drive_sub = rospy.Subscriber('/drive', AckermannDriveStamped, self.drive_callback)
        self.lidar_pub = rospy.Publisher('/lidar', LaserScan, queue_size=10)
        self.image_pub = rospy.Publisher('/image', Image, queue_size=10)
        self.twist_pub = rospy.Publisher('/twist', Twist, queue_size=10)
        #self.max_steering = self.load_param("~max_steering") # create a launch file
        self.last_speed = 0

    def drive_callback(self, drive):
        steering = (drive.drive.steering_angle / self.max_steering) * 180 / math.pi
        throttle = int(drive.drive.speed > self.last_speed)
        twist_msg = Twist()
        self.img, self.twist_msg.linear.x, self.twist_msg.linear.y, self.twist_msg.linear.z, self.last_speed, _, self.laser_msg = self.gym.step((steering, throttle, breaking, reset))
        #print(lidar)

    def pub(self):
        while True:
            if self.lidar_pub is not None:
                self.lidar_pub.publish()
            if self.image_pub is not None:
                self.image_pub.publish()
            if self.twist_pub is not None:
                self.twist_pub.publish()


def main():
    
    rospy.init_node("Wrapper_node", anonymous=True)
    # drive = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=10)
    # a = AckermannDriveStamped()
    # a.drive.speed = 4
    # a.drive.steering_angle = 1
    w = Wrapper()
    T = Thread(target=w.pub, daemon=False)
    T.start()
    # drive.publish(a)
    
    rospy.spin()

if __name__ == '__main__':
    main()