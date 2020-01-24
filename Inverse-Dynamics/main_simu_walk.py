# coding: utf8
from time import clock
import numpy as np 
import time
import pybullet as p
import pybullet_data
import matplotlib.pylab as plt

# import the controller class with its parameters
from walking_PD_controller import controller
import Safety_controller
import EmergencyStop_controller
    

########################################################################
#                        Parameters definition                         #
########################################################################
    
# Simulation parameters
N_SIMULATION = 10000	# number of time steps simulated
dt = 0.001  			#  Time step
t = 0.0  				# time

# Initialize the error for the simulation time
time_error = False

t_list = np.zeros((N_SIMULATION,1))
iterations = np.zeros((N_SIMULATION,1))
torques = np.zeros((N_SIMULATION,8))
des_positions = np.zeros((N_SIMULATION,8))
des_velocities = np.zeros((N_SIMULATION,8))

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

# Fix the base in the world frame
p.createConstraint(robotId, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0.5])

# Enable torque control for revolute joints
jointTorques = [0.0 for m in revoluteJointIndices]

p.setJointMotorControlArray(robotId, revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=jointTorques)

# Set time step for the simulation
p.setTimeStep(dt)


########################################################################
#                             Simulator                                #
########################################################################

myController = controller()

last = clock()
i = 0
#for i in range (N_SIMULATION):
while (i  < N_SIMULATION):    
    if (clock()-last > dt):

        last = clock()
        
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
            mySafetyController = Safety_controller.controller(myController.qdes, myController.vdes)
            myController = mySafetyController
            
        # If the simulation time is too long, controller is switched to a zero torques controller
        time_error = time_error or (clock()-last > 0.003)
        """ if (time_error):
            print("Computation time lasted to long. Switch to a zero torque control")
            myEmergencyStop = EmergencyStop_controller.controller(myController.qdes, myController.vdes)
            myController = myEmergencyStop """
            
        # Retrieve the joint torques from the appropriate controller
        jointTorques = myController.control(qmes, vmes)
    
        for j in range(8):
            torques[i,j] = jointTorques[j]
            des_positions[i,j] = myController.qdes[j]
            des_velocities[i,j] = myController.vdes[j]
            iterations[i,0] = myController.iter
        
        # Time incrementation
        t += dt

        # Set control torque for all joints
        p.setJointMotorControlArray(robotId, revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=jointTorques)

        # Compute one step of simulation
        p.stepSimulation()

        time_spent = clock() - last

        t_list[i,0] = time_spent

        i+=1


# Plot the time of each simulation step
plt.figure(0)
plt.suptitle('iterations controller')
plt.plot(iterations, '*')

plt.figure(1)
plt.suptitle('Time computing')
plt.plot(t_list, 'k+')

plt.figure(2)
plt.suptitle('Torques tracking')
for k in range(8):
    plt.subplot(4,2,k+1)
    plt.plot(torques[:,k])

plt.figure(3)
plt.suptitle('Positions tracking')
for k in range(8):
    plt.subplot(4,2,k+1)
    plt.plot(des_positions[:,k])

plt.figure(4)
plt.suptitle('Velocities tracking')
for k in range(8):
    plt.subplot(4,2,k+1)
    plt.plot(des_velocities[:,k])
plt.show()

p.disconnect()