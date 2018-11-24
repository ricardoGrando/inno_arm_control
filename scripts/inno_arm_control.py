#! /usr/bin/env python
import time
import math
import numpy as np

import rospy
from std_msgs.msg import *
from inno_arm_control_msgs.msg import *

from kinematics import *

def inno_arm_control():

	inno_arm_control_pub = rospy.Publisher("/set_joint_angles", JointMsg, queue_size=10)
	
	rospy.init_node("innor_arm_control", anonymous=False)

	rate = rospy.Rate(10)

	while not rospy.is_shutdown():		

		angles = np.array([0.1,0.1,0.1,0.1,0.1,0.1])  

		for i in range(0, 100):
			J = calc_jacobian(angles)

			J_inv = np.linalg.inv(J)

			delta_angles = J_inv.dot(np.array([0.00, -0.0, -0.1, 0.0, 0.0, 0.0]))
			delta_angles = J_inv.dot(np.array([-0.05, -0.0, 0.1, 0.0, 0.0, 0.0]))
	
			angles += delta_angles

			inno_arm_control_pub.publish([angles[0], angles[1], angles[2], angles[3], angles[4], angles[5]])

			print(angles)
	
			time.sleep(0.1)

		rate.sleep()

	

if __name__ == "__main__":       
    try:
        inno_arm_control()
    except rospy.ROSInterruptException:
        pass
