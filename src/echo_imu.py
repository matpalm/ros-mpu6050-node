#!/usr/bin/env python
import json
import rospy
from sensor_msgs.msg import Imu

def callback(msg):
  av = msg.angular_velocity
  lv = msg.linear_acceleration
  d = {"angular_velocity": {"x": av.x, "y": av.y, "z": av.z},
       "linear_acceleration": {"x": lv.x, "y": lv.y, "z": lv.z}}
  print json.dumps(d)
      
rospy.init_node('imu_echo')
rospy.Subscriber("/imu/data_raw", Imu, callback)

r = rospy.Rate(5)
while not rospy.is_shutdown():
  r.sleep()
                      
