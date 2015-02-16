#!/usr/bin/env python
#In APM, Ground station failsafe enable has been changed from disable to enable always rtl
import os
import sys
import struct
import time,serial,string

import roslib; roslib.load_manifest('roscopter')
import rospy
from std_msgs.msg import String, Header
from std_srvs.srv import *
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from sensor_msgs.msg import Joy


import roscopter.msg

flag=0
ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)
def joy_control(data):
    global pub,flag
    
    
    max_pitch=150
    max_roll=150
    max_thrust=240
    max_yaw=150
     
    if flag<20:
	   pitch=int(1500)
	   roll=int(1500)
	   yaw=int(1500)
	   thrust=int(1000)
	   flag=flag+1
    else:
       pitch=int(1500-(data.axes[1])*max_pitch)
       roll=int(1500-(data.axes[0])*max_roll)
       yaw=int(1500-(data.axes[2])*max_yaw)
       thrust=int(1050+(data.axes[3]+1)*max_thrust)
    
    #print "thrust is %s" % thrust
    
    #header+arm+disarm+manual/automatic
    #roll+pitch+thrust+yaw+channel5
    #channel6+channel7+channel8+payloads
    se_data=struct.pack('>H', 5000)+struct.pack('>H', data.buttons[2])+struct.pack('>H', data.buttons[3])+struct.pack('>H', data.buttons[0])+ \
    struct.pack('>H', roll)+struct.pack('>H', pitch)+struct.pack('>H', thrust)+struct.pack('>H', yaw)+ struct.pack('>H', 0)+struct.pack('>H', 0)+ \
    struct.pack('>H', 0)+struct.pack('>H', 0)+"hello"
    
    ser.write(se_data)
    

def start():
    queue=[]
    rospy.init_node('send')
    rospy.Subscriber("joy", Joy , joy_control)
    while not rospy.is_shutdown():
		x=ser.read(1)
		if x<>"X":
		   queue.append(x)
		else:
			print ''.join(queue)
			queue=[]
    ser.close() 

if __name__ == '__main__':
    start()   
   
#pitch roll yaw 63

#throttle 9-127

#0 63 127
