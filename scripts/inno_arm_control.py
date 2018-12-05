#! /usr/bin/env python
import time
import math
import numpy as np

import rospy
from std_msgs.msg import *
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

inno_arm_control_pub = rospy.Publisher("/set_joint_angles", JointMsg, queue_size=10)

def norm_angles(angles):
	raw_pose = []

	for i in range(0, angles.shape[0]):
		raw_pose.append((float(float(angles[i]) - theta_min[i][1])/float(theta_max[i][1] - theta_min[i][1]))*(theta_max[i][0]-theta_min[i][0]) + theta_min[i][0])
	
	raw_pose[2] = (raw_pose[2] - (raw_pose[1]-theta_min[1][0]))

	return raw_pose

def inverse_kinematics_callback(data):
	# Pegar o target_end_effector da mensagem recebida e converter para array de numpy
	target_position = np.array([data.target_end_effector[0], data.target_end_effector[1], data.target_end_effector[2], data.target_end_effector[3], data.target_end_effector[4], data.target_end_effector[5]])

	# Angulos da posicao inicial
	angles = np.array([0.1,0.1,0.1,0.1,0.1,0.1]) 

	# Taxa de sleep
	rate = rospy.Rate(10)
			 
	while(True):
		# Realizar a cinematica direta para obter a posicao e orientacao cartesiana do end effector
		atual_position = forwardKinematics(angles)

		# Calcular o vetor de distancia entre a posicao cartesiana atual e a desejada
		distance = target_position - atual_position 

		# Verificar se o maior elemento do vetor es menor que 2.0
		# Se o maior elemento do vetor for menor que 2.0, a posicao desejada foi alcancada
		# Se não realizar o processo para ir em direcao da posicao desejada
		# Para pegar o maior elemento do vetor utilize as funcoes mas e abs: max(abs(<array>))

		if (max(abs(target_position - atual_position)) > 2.0):
			
			# No processo para ir em direcao da posicao desejada primeiramente calcular a matriz jacobiana dos angulos
			J = calc_jacobian(angles)
			# Inverter a matriz
			J_inv = np.linalg.inv(J)
			# Calcular o delta do end_effector
			# O MAIOR ELEMENTO DO VETOR DO DELTA DO END EFFECTOR NAO PODE SER MAIOR QUE step_size
			delta_end_effector = ((distance)*data.step_size)/np.max(max(distance))
			print(delta_end_effector)
			# Calcular o delta dos angulos, dado esse delta do end effector
			delta_angles = J_inv.dot(delta_end_effector)
			# Calcular os novos angulos
			angles += delta_angles			
			# Normalizar cada angulo em seus respectivos limites para enviar para os motores. Exemplo: angles[0] entre theta_0_max e theta_0_min
			# Normalizacao = (valor - valor_min)/(valor_max - valor_min)
			print(atual_position)
			raw_angles = norm_angles(angles)
			# Publicar os angulos com o publisher
			inno_arm_control_pub.publish([raw_angles[0], raw_angles[1], raw_angles[2], raw_angles[3], raw_angles[4], raw_angles[5]])
			# Dormir pelo tempo definido
			rate.sleep()
		else:
			print("Arrived!!!")
			rate.sleep()

def inno_arm_control():
	
	# Criar o nodo da aplicacao com o nome: inno_arm_control		
	rospy.init_node("inno_arm_control", anonymous=False)

	# Criar o subscriber para o topico "/set_joint_angles"
	# Esse topico recebe um objeto do tipo CartesianMsg
	# CartesianMsg:
	#   float64[] target_end_effector
	#   float64 step_size
	# Sendo target_end_effector a posição no espaço desejada step_size o tamanho do passo para chegar la
	# O callback do subscriber tem que ser a funcao: inverse_kinematics_callback
	rospy.Subscriber('/cartesian_position', CartesianMsg, inverse_kinematics_callback)
	
	rospy.spin()

####################################### Exemplo para testar a aplicacao #######################################
# rostopic pub -1 /cartesian_position inno_arm_control_msgs/CartesianMsg "target_end_effector: [-17, 2, 20, -2.7, 1.46, 2.67] step_size: 0.1"		

if __name__ == "__main__":       
    try:
        inno_arm_control()
    except rospy.ROSInterruptException:
        pass
