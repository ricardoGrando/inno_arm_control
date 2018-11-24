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


theta_0_max = 2550 # equivale a 90 graus. 
theta_0_min =  750 # equivale a -90 graus 
theta_0_med = int((theta_0_max+theta_0_min)/2.)

theta_1_max = 1250 # equivale a 90 graus. 
theta_1_min =  500 # equivale a 0 graus 
theta_1_med = int((theta_1_max+theta_1_min)/2.)

theta_2_min = 1100 # equivale a 0 graus
theta_2_max = 2100 # equivale a 90 graus.
theta_2_med = int((theta_2_max+theta_2_min)/2.)

theta_3_max = 2500 # equivale a 90 graus.
theta_3_min = 800 # equivale a -90 graus
theta_3_med = int((theta_3_max+theta_3_min)/2.)

theta_4_max = 2400 # equivale a 90 graus.
theta_4_min = 500 # equivale a -90 graus
theta_4_med = int((theta_4_max+theta_4_min)/2.)

theta_5_max = 2400 # equivale a 90 graus
theta_5_min = 500 # equivale a -90 graus
theta_5_med = int((theta_5_max+theta_5_min)/2.)

theta_end_max = 2800
theta_end_min = 2200

theta_max = [[theta_0_max, math.pi/2], [theta_1_max, math.pi/2], [theta_2_max, math.pi/2], [theta_3_max, math.pi/2], [theta_4_max, math.pi/2], [theta_5_max, math.pi/2], [theta_end_max, math.pi/2]]
theta_min = [[theta_0_min, -math.pi/2], [theta_1_min, 0.0], [theta_2_min, 0], [theta_3_min, -math.pi/2], [theta_4_min, -math.pi/2], [theta_5_min, -math.pi/2], [theta_end_min, 0.0]]

def norm_angles(data):
	raw_pose = []

	for i in range(0, len(data.angle)):
		raw_pose.append((float(float(data.angle[i]) - theta_min[i][1])/float(theta_max[i][1] - theta_min[i][1]))*(theta_max[i][0]-theta_min[i][0]) + theta_min[i][0])

	print(raw_pose[1])
        print(raw_pose[2])
        raw_pose[2] = raw_pose[2] - (raw_pose[1]-theta_min[1][0])
        print(raw_pose[2])

	return raw_pose

def callback(data):

	raw_angles = norm_angles(data)

	rospy.loginfo(data)

	k = 1
	while k < 7:
		msg = "#"+str(k)+"P" + str(int(raw_angles[k-1])) + "T100\r\n"
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
