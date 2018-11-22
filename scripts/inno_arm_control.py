import serial
import time
import math
import numpy as np

# serialComm = serial.Serial()
# serialComm.port = '/dev/ttyUSB0'
# serialComm.baudrate = 9600

from kinematics import *

theta_0_max = 2700 # equivale a 90 graus. 0 graus == 1750
theta_0_min =  800 # equivale a -90 graus 

theta_1_max = 1400 # equivale a 90 graus. Talvez um poco menos os 1375
theta_1_min =  550 # equivale a 0 graus 

theta_2_min = 1200 # equivale a 0 graus
theta_2_max = 2200 # equivale a 90 graus. Deveria ser 1975

theta_3_max = 2800 # equivale a 90 graus. 1775 eh 0 graus
theta_3_min = 750 # equivale a -90 graus

theta_4_max = 2400 # equivale a 90 graus. 1450 eh 0 graus
theta_4_min = 500 # equivale a -90 graus

theta_5_max = 2400 # equivale a 90 graus
theta_5_min = 500 # equivale a -90 graus

theta_end_max = 2800
theta_end_min = 2200

theta_max = [[theta_0_max, math.pi/2], [theta_1_max, math.pi/2], [theta_2_max, 0], [theta_3_max, math.pi/2], [theta_4_max, math.pi/2], [theta_5_max, math.pi/2], [theta_end_max, math.pi/2]]
theta_min = [[theta_0_min, -math.pi/2], [theta_1_min, 0.0], [theta_2_min, -math.pi/2], [theta_3_min, -math.pi/2], [theta_4_min, -math.pi/2], [theta_5_min, -math.pi/2], [theta_end_min, 0.0]]

def norm_angles(angles):
	raw_pose = []

	for i in range(0, angles.shape[0]):
		raw_pose.append((float(float(angles[i]) - theta_min[i][1])/float(theta_max[i][1] - theta_min[i][1]))*(theta_max[i][0]-theta_min[i][0]) + theta_min[i][0])

	return raw_pose

# try:
# 	serialComm.open()
# except:
# 	print("deu treta")
# 	exit(0)

angles = np.array([0.1,0.1,0.1,0.1,0.1,0.1])  

for i in range(0, 100):
	J = calc_jacobian(angles)

	J_inv = np.linalg.inv(J)

	delta_angles = J_inv.dot(np.array([0.0, 0.01, 0.0, 0.0, 0.0, 0.0]))

	angles += delta_angles

	print(angles)

	raw_angles = norm_angles(angles)

	print(raw_angles)

	k = 1
	while k < 7:
		msg = "#"+str(k)+"P" + str(int(raw_angles[k-1])) + "T500\r\n"
		print(msg)
		k += 1
		
		# serialComm.write(msg)

		# time.sleep(1)
