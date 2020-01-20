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
		
		self.qdes = q0.copy()
		self.vdes = np.zeros((8,1))
		self.ades = np.zeros((8,1))
		self.error = False
		
		kp_posture = 1.0
		w_posture = 10.0
		
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
		
		## Task definition
		
		# POSTURE Task
		self.postureTask = tsid.TaskJointPosture("task-posture", self.robot)
		self.postureTask.setKp(kp_posture * matlib.ones(8).T) # Proportional gain 
		self.postureTask.setKd(2.0 * np.sqrt(kp_posture) * matlib.ones(8).T) # Derivative gain 
		# Add the task to the HQP with weight = w_posture, priority level = 0 (as real constraint) and a transition duration = 0.0
		self.invdyn.addMotionTask(self.postureTask, w_posture, 0, 0.0)

		
		## TSID Trajectory 
		
		pin.loadReferenceConfigurations(self.model, srdf, False)
		
		q_ref = self.model.referenceConfigurations['straight_standing'] 
		trajPosture = tsid.TrajectoryEuclidianConstant("traj_joint", q_ref)
		
		# Set the trajectory as reference for the posture task
		samplePosture = trajPosture.computeNext()
		self.postureTask.setReference(samplePosture)
		
		## Initialization of the solver

		# Use EiquadprogFast solver
		self.solver = tsid.SolverHQuadProgFast("qp solver")
		# Resize the solver to fit the number of variables, equality and inequality constraints
		self.solver.resize(self.invdyn.nVar, self.invdyn.nEq, self.invdyn.nIn)

				
	####################################################################
	#                      Torque Control method                       #
	####################################################################
	def control(self, qmes, vmes, t):
						
		# Resolution of the HQP problem
		self.HQPData = self.invdyn.computeProblemData(t, self.qdes, self.vdes)
		self.sol = self.solver.solve(self.HQPData)
		
		# Torques, accelerations, velocities and configuration computation
		tau_ff = self.invdyn.getActuatorForces(self.sol)
		self.ades = self.invdyn.getAccelerations(self.sol)
		self.vdes += self.ades * dt
		self.qdes = pin.integrate(self.model, self.qdes, self.vdes * dt)
		
		# Torque PD controller
		P = 10.0
		D = 0.2
		torques = P * (self.qdes - qmes) + D * (self.vdes - vmes) + tau_ff
		
		# Saturation to limit the maximal torque
		t_max = 2.5
		tau = np.maximum(np.minimum(torques, t_max * np.ones((8,1))), -t_max * np.ones((8,1)))
		
		self.error = self.error or (self.sol.status!=0) or (qmes[0] < -np.pi/2) or (qmes[2] < -np.pi/2) or (qmes[4] < -np.pi/2) or (qmes[6] < -np.pi/2) or (qmes[0] > np.pi/2) or (qmes[2] > np.pi/2) or (qmes[4] > np.pi/2) or (qmes[6] > np.pi/2)
		
		if (self.error): print("Status of the solution : ", self.sol.status)
		
		return tau

# Parameters for the controller

dt = 0.001				# controller time step

q0 = np.zeros((8,1))	# initial configuration

omega = 1.0				# sinus pulsation
