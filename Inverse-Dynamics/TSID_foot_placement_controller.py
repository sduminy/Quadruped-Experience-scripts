# coding: utf8


########################################################################
#                                                                      #
#          Control law : tau = P(q*-q^) + D(v*-v^) + tau_ff            #
#                                                                      #
########################################################################

import pinocchio as pin
import numpy as np
import numpy.matlib as matlib
import tsid

pin.switchToNumpyMatrix()


########################################################################
#            Class for a PD with feed-forward Controller               #
########################################################################

class controller:
	
	def __init__(self, q0, omega, t):
		
		self.omega = omega
		self.qdes = q0.copy()
		self.vdes = np.zeros((8,1))
		self.ades = np.zeros((8,1))
		self.error = False
		
		kp_foot = 10.0
		w_foot = 10.0
		
		
		########################################################################
		#             Definition of the Model and TSID problem                 #
		########################################################################
		
		## Set the paths where the urdf and srdf file of the robot are registered

		modelPath = "/opt/openrobots/lib/python3.5/site-packages/../../../share/example-robot-data/robots"
		urdf = modelPath + "/solo_description/robots/solo.urdf"
		srdf = modelPath + "/solo_description/srdf/solo.srdf"
		vector = pin.StdVec_StdString()
		vector.extend(item for item in modelPath)

		## Create the robot wrapper from the urdf model (without the free flyer)

		self.robot = tsid.RobotWrapper(urdf, vector, False)
		
		self.model = self.robot.model()
		
		## Creation of the Invverse Dynamics HQP problem using the robot
		## accelerations (base + joints) and the contact forces

		self.invdyn = tsid.InverseDynamicsFormulationAccForce("tsid", self.robot, False)
		# Compute the problem data with a solver based on EiQuadProg
		self.invdyn.computeProblemData(t, self.qdes, self.vdes)
		# Get the initial data
		self.data = self.invdyn.data()
		
		# Task definition
		self.FRfootTask = tsid.TaskSE3Equality("FR-foot-placement", self.robot, 'FR_FOOT')
		self.FRfootTask.setKp(kp_foot * matlib.ones(6).T)
		self.FRfootTask.setKd(2.0 * np.sqrt(kp_foot) * matlib.ones(6).T)
		self.FRfootTask.setMask(np.matrix([[1,0,1, 0,0,0]]).T) #set a mask allowing only the transation upon x and z-axis
		self.FRfootTask.useLocalFrame(False)
		# Add the task to the HQP with weight = w_foot, priority level = 0 (as real constraint) and a transition duration = 0.0
		self.invdyn.addMotionTask(self.FRfootTask, w_foot, 1, 0.0)
		
		
		## TSID Trajectory 
		
		pin.forwardKinematics(self.model, self.data, self.qdes)
		pin.updateFramePlacements(self.model, self.data)
	
		self.FR_foot_ref = self.robot.framePosition(self.data, self.model.getFrameId('FR_FOOT'))
		
		FRgoalx = self.FR_foot_ref.translation[0,0] + 0.1 
		FRgoalz = self.FR_foot_ref.translation[2,0] + 0.1
		
		self.FR_foot_goal = self.FR_foot_ref.copy()
		self.FR_foot_goal.translation = np.matrix([FRgoalx, self.FR_foot_ref.translation[1,0], FRgoalz]).T
		
		self.trajFRfoot = tsid.TrajectorySE3Constant("traj_FR_foot", self.FR_foot_goal)
		
		# Set the trajectory as reference for the foot positionning task
		self.sampleFoot = self.trajFRfoot.computeNext()
		self.FRfootTask.setReference(self.sampleFoot)
		
		## Initialization of the solver

		# Use EiquadprogFast solver
		self.solver = tsid.SolverHQuadProgFast("qp solver")
		# Resize the solver to fit the number of variables, equality and inequality constraints
		self.solver.resize(self.invdyn.nVar, self.invdyn.nEq, self.invdyn.nIn)

				
	####################################################################
	#                      Torque Control method                       #
	####################################################################
	def control(self, qmes, vmes, t):
		
		pin.forwardKinematics(self.model, self.data, self.qdes)
		pin.updateFramePlacements(self.model, self.data)
	
		self.FR_foot_mes = self.robot.framePosition(self.data, self.model.getFrameId('FR_FOOT'))
					
		"""if (t>5.0):
			FRgoalx = 0.05 * np.cos(self.omega*t) + (self.FR_foot_ref.translation[0,0] + 0.2) 
			FRgoalz = 0.05 * np.sin(self.omega*t) + (self.FR_foot_ref.translation[2,0] + 0.2)
			
			self.FR_foot_goal.translation = np.matrix([FRgoalx, self.FR_foot_ref.translation[1,0], FRgoalz]).T
		
			self.trajFRfoot = tsid.TrajectorySE3Constant("traj_FR_foot", self.FR_foot_goal)
			
			self.sampleFoot = self.trajFRfoot.computeNext()
			self.FRfootTask.setReference(self.sampleFoot)
		"""	
		# Resolution of the HQP problem
		HQPData = self.invdyn.computeProblemData(t, self.qdes, self.vdes)
		self.sol = self.solver.solve(HQPData)
		
		# Torques, accelerations, velocities and configuration computation
		tau_ff = self.invdyn.getActuatorForces(self.sol)
		self.ades = self.invdyn.getAccelerations(self.sol)
		self.vdes += self.ades * dt
		self.qdes = pin.integrate(self.model, self.qdes, self.vdes * dt)
		
		# Torque PD controller
		P = 50.0
		D = 0.2
		torques = P * (self.qdes - qmes) + D * (self.vdes - vmes) + tau_ff
		
		# Saturation to limit the maximal torque
		t_max = 2.5
		tau = np.maximum(np.minimum(torques, t_max * np.ones((8,1))), -t_max * np.ones((8,1)))
		
		self.error = self.error or (self.sol.status!=0) or (qmes[0] < -np.pi/2) or (qmes[2] < -np.pi/2) or (qmes[4] < -np.pi/2) or (qmes[6] < -np.pi/2) or (qmes[0] > np.pi/2) or (qmes[2] > np.pi/2) or (qmes[4] > np.pi/2) or (qmes[6] > np.pi/2)
		
		return tau.flatten()

# Parameters for the controller

dt = 0.001				# controller time step

q0 = np.matrix([0.8, -1.6, 0.8, -1.6, -0.8, 1.6, -0.8, 1.6]).T	# initial configuration

omega = 1.0				# sinus pulsation
