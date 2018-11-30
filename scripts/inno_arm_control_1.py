#! /usr/bin/env python
import math
import numpy as np
import rospy

from inno_arm_control_msgs.msg import *
from kinematics import *

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
theta_min = [[theta_0_min, -math.pi/2], [theta_1_min, 0.0], [theta_2_min, 0.0], [theta_3_min, -math.pi/2], [theta_4_min, -math.pi/2], [theta_5_min, -math.pi/2], [theta_end_min, 0.0]]

def norm_angles(angles):
	raw_pose = []

	for i in range(0, angles.shape[0]):
		raw_pose.append((float(float(angles[i]) - theta_min[i][1])/float(theta_max[i][1] - theta_min[i][1]))*(theta_max[i][0]-theta_min[i][0]) + theta_min[i][0])
	
	raw_pose[2] = (raw_pose[2] - (raw_pose[1]-theta_min[1][0]))

	return raw_pose

def inno_arm_control():

	# Criar o nodo da aplicacao com o nome: inno_arm_control		
	rospy.init_node("inno_arm_control", anonymous=False)

	# Criar o publisher para os angulos
	# O nome do topico es "/set_joint_angles" e recebe um objeto do tipo JointMsg
	# JointMsg:
	#   float64[] angle 
	inno_arm_control_pub = rospy.Publisher("/set_joint_angles", JointMsg, queue_size=10)

	# Inicilaizar a taxa de sleep em 0.1 segundos
	rate = rospy.Rate(10)
	
	while not rospy.is_shutdown():

		# Angulos da posicao inicial e delta do end effector
		angles = np.array([0.1,0.1,0.1,0.1,0.1,0.1]) 		
		# A posicao catesiana inicial desse angulos es mas o menos: [-17.20506684   2.05361388  14.11294844  -2.77581853   1.41865181 2.67694514]

		# Delta do end_effector 		
		delta_end_effector = np.array([-0.05, -0.0, 0.1, 0.0, 0.0, 0.0])		 

		for i in range(0, 100):
			# Calcular a matriz jacobiana desses angulos
			J = calc_jacobian(angles)
			# Inverter a matriz
			J_inv = np.linalg.inv(J)
			# Calcular os delta dos angulos
			delta_angles = J_inv.dot(delta_end_effector)
			# Somar com os angulos com o delta dos angulos
			angles += delta_angles

			print(forwardKinematics(angles))

			# Normalizar cada angulo em seus respectivos limites para enviar para os motores. Exemplo: angles[0] entre theta_0_max e theta_0_min
			# Normalizacao = (valor - valor_min)/(valor_max - valor_min)
			raw_angles = norm_angles(angles)

			# Publicar os angulos com o publisher
			inno_arm_control_pub.publish([raw_angles[0], raw_angles[1], raw_angles[2], raw_angles[3], raw_angles[4], raw_angles[5]])
			
			# Dormir pelo tempo definido
			rate.sleep()	

		# Realizar esse processo 100 vezes até chegar aproximadamente na posicao: [-22.28313997   1.38575526  24.08532308  -2.7020912    1.46859977   2.6766997 ]

if __name__ == "__main__":       
    try:
        inno_arm_control()
    except rospy.ROSInterruptException:
        pass
