# coding: utf8

#####################
## LOADING MODULES ##
#####################

# PyBullet modules
import pybullet as p # PyBullet simulator
import pybullet_data

# Pinocchio modules
import pinocchio as pin # Pinocchio library
from pinocchio.robot_wrapper import RobotWrapper # Robot Wrapper to load an URDF in Pinocchio

# Other modules
import time # Time module to sleep()
from initialization_simulation import configure_simulation, getPosVelJoints # Functions to initialize the simulation and retrieve joints positions/velocities
from walking_controller import c_walking_IK_bezier # Controller functions
import matplotlib.pylab as plt 
<<<<<<< HEAD
import numpy as np
=======
>>>>>>> db692571595ad962571f83ff05c98ef88268a8a4

####################
## INITIALIZATION ##
####################

<<<<<<< HEAD
dt = 0.001 # time step of the simulation
realTimeSimulation = False # If True then we will sleep in the main loop to have a 1:1 ratio of (elapsed real time / elapsed time in the simulation)
=======
dt = 0.001 # time step of the simulation
realTimeSimulation = False # If True then we will sleep in the main loop to have a 1:1 ratio of (elapsed real time / elapsed time in the simulation)
>>>>>>> db692571595ad962571f83ff05c98ef88268a8a4
enableGUI = True # enable PyBullet GUI or not
robotId, solo, revoluteJointIndices = configure_simulation(dt, enableGUI)

###############
## MAIN LOOP ##
###############

t_list = []
T = 0.3 			# period of the foot trajectory
Q = np.zeros((int(T/dt),8))
V = np.zeros((int(T/dt),8))

<<<<<<< HEAD
for i in range(10000): # run the simulation during dt * i_max seconds (simulation time)
=======
for i in range(1000): # run the simulation during dt * i_max seconds (simulation time)
>>>>>>> db692571595ad962571f83ff05c98ef88268a8a4
	
	t_start = time.time()

	# Get position and velocity of all joints in PyBullet (free flying base + motors)
	q, qdot = getPosVelJoints(robotId, revoluteJointIndices)

	# Call controller to get torques for all joints
	jointTorques, qa_ref, qa_dot_ref = c_walking_IK_bezier(q, qdot, dt, solo, dt*i)
	
<<<<<<< HEAD
	if(i>=int(8*T/dt) and i<=int(9*T/dt)):
		for k in range(8):
			Q[i%int(T/dt),k] = qa_ref[k]
			V[i%int(T/dt),k] = qa_dot_ref[k]
	"""if(i==1+int(9*T/dt)):
		for k in range(8):
			Q[int(T/dt),k] = qa_ref[k]
			V[int(T/dt),k] = qa_dot_ref[k]"""
	
=======
	if(i<int(T/dt)):
		for k in range(8):
			Q[i,k] = qa_ref[k]
			V[i,k] = qa_dot_ref[k]
>>>>>>> db692571595ad962571f83ff05c98ef88268a8a4

	# Set control torques for all joints in PyBullet
	p.setJointMotorControlArray(robotId, revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=jointTorques)

	# Compute one step of simulation
	p.stepSimulation()
	
	t_spent = time.time() - t_start
		
	t_list.append(t_spent)


<<<<<<< HEAD
np.savez('/local/users/sduminy/Quadruped-Experience-scripts/Inverse-Kinematics/reference_q_v', qdes=Q, vdes=V)
=======
np.savez('/home/ada/Desktop/Script_Segolene_XP/Quadruped-Experience-scripts/Inverse-Dynamics/reference_q_v', qdes=Q, vdes=V)
>>>>>>> db692571595ad962571f83ff05c98ef88268a8a4


""" plt.figure(1)
plt.plot(t_list, 'k+')
plt.show() """

# Shut down the PyBullet client
p.disconnect()
