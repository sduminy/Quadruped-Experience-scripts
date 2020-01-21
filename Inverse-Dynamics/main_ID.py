# coding: utf8

import numpy as np 
import time

# import the controller class with its parameters
from P_controller import controller, dt, q0, omega
import Relief_controller
import EmergencyStop_controller


# Flag robot's use to clean the CPU use when using the robot
useRobot = False	# false when the robot is not used


if useRobot :
	import libmaster_board_sdk_pywrap as mbs

if not useRobot:
	import pybullet as p
	import pybullet_data
	import matplotlib.pylab as plt
	t_list = []

########################################################################
#                        Parameters definition                         #
########################################################################
	
# Simulation parameters
N_SIMULATION = 10000	# number of time steps simulated

t = 0.0  				# time

# Initialize the error for the simulation time
time_error = False

if useRobot:
	N_SLAVES = 6  # Maximum number of controled drivers
	N_SLAVES_CONTROLED = 2  # Current number of controled drivers

	iq_sat = 1.0  # Maximum amperage (A)
	init_pos = [0.0 for i in range(N_SLAVES * 2)]  # List that will store the initial position of motors
	state = 0  # State of the system (ready (1) or not (0))

	name_interface = enp4s2f0 # Name of the ethernet interface : type ifconfig
	
	robot_if = mbs.MasterBoardInterface(name_interface) # Robot interface
	robot_if.Init()  # Initialization of the interface between the computer and the master board
	
	for i in range(N_SLAVES_CONTROLED):  # We enable each controler driver and its two associated motors
		robot_if.GetDriver(i).motor1.SetCurrentReference(0)
		robot_if.GetDriver(i).motor2.SetCurrentReference(0)
		robot_if.GetDriver(i).motor1.Enable()
		robot_if.GetDriver(i).motor2.Enable()
		robot_if.GetDriver(i).EnablePositionRolloverError()
		robot_if.GetDriver(i).SetTimeout(5)
		robot_if.GetDriver(i).Enable()

	qmes = np.zeros((8,1))
	vmes = np.zeros((8,1))
	K = 1.0		# proportionnal gain to convert torques to current	
	
	
if not useRobot:

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

	""" # Initialize the joint configuration to the position straight_standing
	initial_joint_positions = [0.8, -1.6, 0.8, -1.6, -0.8, 1.6, -0.8, 1.6]
	for i in range (len(initial_joint_positions)):
		p.resetJointState(robotId, revoluteJointIndices[i], initial_joint_positions[i])
	"""
	
	# Initialize the FR_hip configuration to 0.8
	#p.resetJointState(robotId, revoluteJointIndices[0], 0.8)

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


for i in range (N_SIMULATION):
	
	time_start = time.time()
	
	if useRobot :

		robot_if.ParseSensorData()  # Read sensor data sent by the masterboard

		if (state == 0):  # If the system is not ready
			state = 1
			for i in range(N_SLAVES_CONTROLED * 2):  # Check if all motors are enabled and ready
				if not (robot_if.GetMotor(i).IsEnabled() and robot_if.GetMotor(i).IsReady()):
					state = 0
				init_pos[i] = robot_if.GetMotor(i).GetPosition()
				t = 0.0
		else:  # If the system is ready
			for i in range(N_SLAVES_CONTROLED * 2):
				if robot_if.GetMotor(i).IsEnabled():
					qmes[i] = robot_if.GetMotor(i).GetPosition()
					vmes[i] = robot_if.GetMotor(i).GetVelocity()


	if not useRobot:

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
	
	# Time incrementation
	t += dt
	
	if useRobot:
		if (state==1):
			for i in range(N_SLAVES_CONTROLED * 2):
				if robot_if.GetMotor(i).IsEnabled():
					cur = K * jointTorques
					if (cur > iq_sat):  # Check saturation
						cur = iq_sat
					if (cur < -iq_sat):
						cur = -iq_sat
					robot_if.GetMotor(i).SetCurrentReference(cur)  # Set reference currents
		robot_if.SendCommand()  # Send the reference currents to the master board


	if not useRobot:

		# Set control torque for all joints
		p.setJointMotorControlArray(robotId, revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=jointTorques)

		# Compute one step of simulation
		p.stepSimulation()

		time_spent = time.time() - time_start

		t_list.append(time_spent)


if useRobot:
	robot_if.Stop()  # Shut down the interface between the computer and the master board

if not useRobot:

	# Plot the time of each simulation step
	plt.figure(1)
	plt.plot(t_list, 'k+')
	plt.show()

	p.disconnect()
