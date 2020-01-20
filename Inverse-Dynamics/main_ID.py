# coding: utf8

import pybullet as p 
import numpy as np 
import pinocchio as pin
import pybullet_data
import time
# import the controller class with its parameters
from TSID_foot_placement_controller import controller, dt, q0, omega
import Relief_controller
import EmergencyStop_controller

########################################################################
#                        Parameters definition                         #
########################################################################
	
# Simulation parameters
N_SIMULATION = 10000	# number of time steps simulated

t = 0.0  				# time

# Set the simulation in real time
realTimeSimulation = True

# Initialize the error for the simulation time
time_error = False


########################################################################
#                              PyBullet                                #
########################################################################

# Start the client for PyBullet
physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version

# Load horizontal plane
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")

# Set the gravity
p.setGravity(0,0,-9.81)

# Load Quadruped robot
robotStartPos = [0,0,0.5] 
robotStartOrientation = p.getQuaternionFromEuler([0,0,0])
p.setAdditionalSearchPath("/opt/openrobots/share/example-robot-data/robots/solo_description/robots")
robotId = p.loadURDF("solo.urdf",robotStartPos, robotStartOrientation)

# Disable default motor control for revolute joints
revoluteJointIndices = [0,1, 3,4, 6,7, 9,10]
p.setJointMotorControlArray(robotId, jointIndices = revoluteJointIndices, controlMode = p.VELOCITY_CONTROL,targetVelocities = [0.0 for m in revoluteJointIndices], forces = [0.0 for m in revoluteJointIndices])

# Initialize the joint configuration to the position straight_standing
initial_joint_positions = [0.8, -1.6, 0.8, -1.6, -0.8, 1.6, -0.8, 1.6]
for i in range (len(initial_joint_positions)):
	p.resetJointState(robotId, revoluteJointIndices[i], initial_joint_positions[i])

# Enable torque control for revolute joints
jointTorques = [0.0 for m in revoluteJointIndices]

p.setJointMotorControlArray(robotId, revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=jointTorques)

# Fix the base in the world frame
p.createConstraint(robotId, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0.5])

# Set time step for the simulation
p.setTimeStep(dt)


########################################################################
#                             Simulator                                #
########################################################################

myController = controller(q0, omega, t)
myReliefController = Relief_controller.controller()
myEmergencyStop = EmergencyStop_controller.controller()

Qmes = [[],[],[],[],[],[],[],[]]
Vmes = [[],[],[],[],[],[],[],[]]
Tau = [[],[],[],[],[],[],[],[]]

t_list = []

pos_error = [[],[]]

for i in range (N_SIMULATION):
	
	if realTimeSimulation:	
		time_start = time.time()
		
	####################################################################
	#                 Data collection from PyBullet                    #
	####################################################################
	
	jointStates = p.getJointStates(robotId, revoluteJointIndices) # State of all joints
	
	# Joints configuration and velocity vector
	qmes = np.vstack((np.array([[jointStates[i_joint][0] for i_joint in range(len(jointStates))]]).T))
	vmes = np.vstack((np.array([[jointStates[i_joint][1] for i_joint in range(len(jointStates))]]).T))
	
	####################################################################
	#                Select the appropriate controller 				   #
	#								&								   #
	#				Load the joint torques into the robot			   #
	####################################################################
	
	# If the limit bounds are reached, controller is switched to a pure derivative controller
	if(myController.error):
		print("Safety bounds reached. Switch to a safety controller")
		myController = myReliefController
		
	# If the simulation time is too long, controller is switched to a zero torques controller
	time_error = time_error or (time.time()-time_start > 0.003)
	if (time_error):
		print("Computation time lasted to long. Switch to a zero torque control")
		myController = myEmergencyStop
		
	# Retrieve the joint torques from the appropriate controller
	jointTorques = myController.control(qmes, vmes, t)
	
	pos_error[0].append(float(myController.FR_foot_goal.translation[0,] - myController.FR_foot_mes.translation[0,]))
		
	pos_error[1].append(float(myController.FR_foot_goal.translation[2,] - myController.FR_foot_mes.translation[2,]))
	
	
	# Set control torque for all joints
	p.setJointMotorControlArray(robotId, revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=jointTorques)
	
	# Track the trajectories
	
	for i in range(8):
		Qmes[i].append(qmes[i])
		Vmes[i].append(vmes[i])
		Tau[i].append(jointTorques[i,0])
	
	# Compute one step of simulation
	p.stepSimulation()
	
	# Time incrementation
	t += dt
	
	if realTimeSimulation:
		time_spent = time.time() - time_start
		if time_spent < dt:
			time.sleep(dt-time_spent)	# ensure the simulation runs in real time
	
	t_list.append(time_spent)

if(myController.error):
		print("Status of the solution : ", myController.sol.status)

## Plot the tracking of the trajectories

import matplotlib.pylab as plt

plt.figure(1)

plt.subplot(3,1,1)
for i in range(8):
		plt.plot(Qmes[i], '-')
plt.grid()
plt.title("Configuration tracking")

plt.subplot(3,1,2)
for i in range(8):
	plt.plot(Vmes[i], '-')
plt.grid()
plt.title("Velocity tracking")

plt.subplot(3,1,3)

for i in range(8):
	plt.plot(Tau[i], '-')
plt.grid()
plt.title("Torques tracking")

plt.show()	

plt.figure(2)
plt.plot(t_list, 'k+')

plt.show()

