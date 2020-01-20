# coding: utf8

import pybullet as p 
import numpy as np 
import pinocchio as pin
import pybullet_data
import time
# import the controller class with its parameters
from TSID_contacts_and_posture_controller import controller, dt, q0, omega
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

# Initialize the robot in a specific configuration
#p.resetJointStatesMultiDof(robotId, revoluteJointIndices, q0)						

# Enable torque control for revolute joints
jointTorques = [0.0 for m in revoluteJointIndices]

p.setJointMotorControlArray(robotId, revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=jointTorques)

# Set time step for the simulation
p.setTimeStep(dt)


########################################################################
#                             Simulator                                #
########################################################################

myController = controller(q0, omega, t)
myReliefController = Relief_controller.controller_12dof()
myEmergencyStop = EmergencyStop_controller.controller()

Qmes = [[],[],[],[],[],[],[],[]]
Vmes = [[],[],[],[],[],[],[],[]]
Tau = [[],[],[],[],[],[],[],[]]

t_list = []

for i in range (N_SIMULATION):
	
	if realTimeSimulation:	
		time_start = time.time()
		
	####################################################################
	#                 Data collection from PyBullet                    #
	####################################################################
	
	jointStates = p.getJointStates(robotId, revoluteJointIndices) # State of all joints
	baseState   = p.getBasePositionAndOrientation(robotId)
	
	# Joints configuration and velocity vector
	qmes8 = np.vstack((np.array([baseState[0]]).T, np.array([baseState[1]]).T, np.array([[jointStates[i_joint][0] for i_joint in range(len(jointStates))]]).T))
	vmes8 = np.vstack((np.zeros((6,1)), np.array([[jointStates[i_joint][1] for i_joint in range(len(jointStates))]]).T))
	
	# Conversion (from 8 to 12 DOF) for TSID computation 
	qmes12 = np.concatenate((qmes8[:7], np.matrix([0.]), qmes8[7:9], np.matrix([0.]), qmes8[9:11], np.matrix([0.]), qmes8[11:13], np.matrix([0.]), qmes8[13:15]))
	vmes12 = np.concatenate((vmes8[:6], np.matrix([0.]), vmes8[6:8], np.matrix([0.]), vmes8[8:10], np.matrix([0.]), vmes8[10:12], np.matrix([0.]), vmes8[12:14]))
	
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
	time_error = time_error or (time.time()-time_start > 0.001)
	if (time_error):
		print("Computation time lasted to long. Switch to a zero torque control")
		myController = myEmergencyStop
		
	# Retrieve the joint torques from the appropriate controller
	jointTorques = myController.control(qmes12, vmes12, t)
		
	# Set control torque for all joints
	p.setJointMotorControlArray(robotId, revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=jointTorques)
	
	# Track the trajectories
	
	for i in range(8):
		Qmes[i].append(qmes8[i+7])
		Vmes[i].append(vmes8[i+6])
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

