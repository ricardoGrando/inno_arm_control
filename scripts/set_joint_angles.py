#! /usr/bin/env python
import serial
import time
import math
import numpy as np

import rospy

from std_msgs.msg import *
from inno_arm_control_msgs.msg import *

serialComm = serial.Serial()
serialComm.port = '/dev/ttyUSB0'
serialComm.baudrate = 9600

def callback(data):	

	rospy.loginfo(data)

	k = 1
	while k < 7:
		msg = "#"+str(k)+"P" + str(int(data.angle[k-1])) + "T100\r\n"
		#print(msg)
		k += 1
		
		serialComm.write(msg)


def set_joints_angles():
    rospy.init_node("set_joint_angles", anonymous=False)

    rospy.Subscriber("/set_joint_angles", JointMsg, callback)

    rospy.spin()    

if __name__ == "__main__":
    try:
	serialComm.open()
    except:
	print("Error de serial")
	exit(0)
    
    try:
        set_joints_angles()
    except rospy.ROSInterruptException:
        pass
