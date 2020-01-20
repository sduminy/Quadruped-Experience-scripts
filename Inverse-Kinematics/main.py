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
realTimeSimulation = True # If True then we will sleep in the main loop to have a 1:1 ratio of (elapsed real time / elapsed time in the simulation)
enableGUI = False # enable PyBullet GUI or not
robotId, solo, revoluteJointIndices = configure_simulation(dt, enableGUI)

###############
## MAIN LOOP ##
###############
jointTorques_list = [[],[],[],[],[],[],[],[]]

xdes=[]
zdes=[]
xmes=[]
zmes=[]
	
for i in range(10000): # run the simulation during dt * i_max seconds (simulation time)
   
	# Time at the start of the loop
	if realTimeSimulation:
		t_start = time.time()

	# Get position and velocity of all joints in PyBullet (free flying base + motors)
	q, qdot = getPosVelJoints(robotId, revoluteJointIndices)

	# Call controller to get torques for all joints
	jointTorques, a, b, c, d = c_walking_IK_bezier(q, qdot, dt, solo, dt*i)
	for j in range(8):
		jointTorques_list[j].append(float(jointTorques[j]))
	xdes.append(float(a))
	zdes.append(float(b))
	xmes.append(float(c))
	zmes.append(float(d))
	
	# Set control torques for all joints in PyBullet
	p.setJointMotorControlArray(robotId, revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=jointTorques)

	# Compute one step of simulation
	p.stepSimulation()
	
	# Sleep to get a real time simulation
	if realTimeSimulation:
		t_spent = time.time() - t_start
		if t_spent < dt:
			time.sleep(dt - t_spent)

"""			
T = 0.3 			# period of the foot trajectory
DT = int(T/dt)		# number of iteration for one period
Q = Q_list[2*DT:4*DT+1] # Q is the list of the configurations for one trajectory cycle (the 2nd one to avoid the singularities of the 1st one)
N=2*DT-1
"""

import matplotlib.pylab as plt 
"""
plt.figure(1)
plt.plot(jointTorques_list[0])
plt.title("Torques applied during the walk")
plt.show()
"""
"""
print('q[{}][8] = '.format(N+1), end='')
print('{', end='') 
for i in range(N+1): 
	print ('{',"{},{},{},{},{},{},{},{}".format(Q[i][0,0],Q[i][1,0],Q[i][2,0],Q[i][3,0],Q[i][4,0],Q[i][5,0],Q[i][6,0],Q[i][7,0]),'},') 
print ('{',"{},{},{},{},{},{},{},{}".format(Q[N+1][0,0],Q[N+1][1,0],Q[N+1][2,0],Q[N+1][3,0],Q[N+1][4,0],Q[N+1][5,0],Q[N+1][6,0],Q[N+1][7,0]),'}', end='')
print('};') 
"""

# Shut down the PyBullet client
p.disconnect()
