# coding: utf8

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

####################
## INITIALIZATION ##
####################

dt = 0.001 # time step of the simulation
realTimeSimulation = False # If True then we will sleep in the main loop to have a 1:1 ratio of (elapsed real time / elapsed time in the simulation)
enableGUI = False # enable PyBullet GUI or not
robotId, solo, revoluteJointIndices = configure_simulation(dt, enableGUI)

###############
## MAIN LOOP ##
###############

t_list = []

for i in range(10000): # run the simulation during dt * i_max seconds (simulation time)
	
	t_start = time.time()

	# Get position and velocity of all joints in PyBullet (free flying base + motors)
	q, qdot = getPosVelJoints(robotId, revoluteJointIndices)

	# Call controller to get torques for all joints
	jointTorques= c_walking_IK_bezier(q, qdot, dt, solo, dt*i)
		
	# Set control torques for all joints in PyBullet
	#p.setJointMotorControlArray(robotId, revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=jointTorques)

	# Compute one step of simulation
	#p.stepSimulation()
	
	t_spent = time.time() - t_start
		
	t_list.append(t_spent)


import matplotlib.pylab as plt 

plt.figure(1)
plt.plot(t_list, 'k+')
plt.show()

# Shut down the PyBullet client
p.disconnect()
