## Inno Arm Control

# To Run the interface between the computer and the arm
rosrun inno_arm_control set_joint_angles.py

# To run the application to control the arm
rosrun inno_arm_control inno_arm_control.py 

# Example of the usase
rostopic pub -1 /cartesian_position inno_arm_control_msgs/CartesianMsg "target_end_effector: [-17, 2, 20, -2.7, 1.46, 2.67] step_size: 0.1"		
