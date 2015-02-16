#!/usr/bin/env python
#In APM, Ground station failsafe enable has been changed from disable to enable always rtl
import os
import sys
import struct
import time

import roslib; roslib.load_manifest('roscopter')
import rospy
from std_msgs.msg import String, Header
from std_srvs.srv import *
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from sensor_msgs.msg import Joy


import roscopter.msg
from geometry_msgs.msg import Twist

def joy_control(data):
    global pub
    max_pitch=200
    max_roll=200
    max_thrust=150
    if flight_mode==0:
       pitch=1500+(data.axes[1])*max_pitch
       roll=1500-(data.axes[0])*max_roll
       thrust=1000+(data.axes[2]+1)*max_thrust
    else:
		pitch=0
		roll=0
		thrust=0
    pub.publish([pitch,roll,thrust,1522,flight_mode,1918,1498,1498])
   # print(thrust)
    #print(data.axes[1])

def emergency(data):
	global flight_mode
	flight_mode=1

global flight_mode
flight_mode=0
  
def start():
    global pub
    rospy.init_node('command')
    
    pub=rospy.Publisher('send_rc', roscopter.msg.RC)
    rospy.Subscriber("joy", Joy , joy_control)
    rospy.Subscriber("cmd_vel", Twist , emergency)
    
    rospy.spin()

if __name__ == '__main__':
    start()   
   
#pitch roll yaw 63

#throttle 9-127

#0 63 127
