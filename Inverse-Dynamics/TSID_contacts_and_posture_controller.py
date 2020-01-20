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
		self.vdes = np.zeros((18,1))
		self.ades = np.zeros((18,1))
		self.error = False
		
		kp_posture = 10.0		# proportionnal gain of the posture task
		w_posture = 1.0			# weight of the posture task
		
		kp_lock = 0.0			# proportionnal gain of the lock task
		w_lock = 10.0			# weight of the lock task

		# For the contacts
		mu = 0.3  				# friction coefficient
		fMin = 1.0				# minimum normal force
		fMax = 100.0  			# maximum normal force

		w_forceRef = 1e-3		# weight of the forces regularization
		kp_contact = 0.0		# proportionnal gain for the contacts

		foot_frames = ['HL_FOOT', 'HR_FOOT', 'FL_FOOT', 'FR_FOOT']  # tab with all the foot frames names
		contactNormal = np.matrix([0., 0., 1.]).T  # direction of the normal to the contact surface

		########################################################################
		#             Definition of the Model and TSID problem                 #
		########################################################################
		
		## Set the paths where the urdf and srdf file of the robot are registered

		modelPath = "/opt/openrobots/lib/python3.5/site-packages/../../../share/example-robot-data/robots"
		urdf = modelPath + "/solo_description/robots/solo12.urdf"
		srdf = modelPath + "/solo_description/srdf/solo.srdf"
		vector = pin.StdVec_StdString()
		vector.extend(item for item in modelPath)

		## Create the robot wrapper from the urdf model (without the free flyer)

		self.robot = tsid.RobotWrapper(urdf, vector, pin.JointModelFreeFlyer(), False)
		
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
		self.postureTask.setKp(kp_posture * matlib.ones(self.robot.nv-6).T) # Proportional gain 
		self.postureTask.setKd(2.0 * np.sqrt(kp_posture) * matlib.ones(self.robot.nv-6).T) # Derivative gain 
		# Add the task to the HQP with weight = w_posture, priority level = 0 (as real constraint) and a transition duration = 0.0
		self.invdyn.addMotionTask(self.postureTask, w_posture, 1, 0.0)
		
		# LOCK Task
		self.lockTask = tsid.TaskJointPosture("task-lock-shoulder", self.robot)
		self.lockTask.setKp(kp_lock * matlib.ones(self.robot.nv-6).T) 
		self.lockTask.setKd(2.0 * np.sqrt(kp_lock) * matlib.ones(self.robot.nv-6).T) 
		mask = np.matrix(np.zeros(self.robot.nv-6))	# Add a mask to take into account only the shoulders joints
		for i in [0, 3, 6, 9]:
			mask[0,i] = 1
		self.lockTask.mask(mask.T)
		self.invdyn.addMotionTask(self.lockTask, w_lock, 0, 0.0) # Add the task as real constraint (priority level = 0)
		
		
		## CONTACTS
		
		self.contacts = 4*[None]

		for i, name in enumerate(foot_frames):
			self.contacts[i] = tsid.ContactPoint(name, self.robot, name, contactNormal, mu, fMin, fMax)
			self.contacts[i].setKp(kp_contact * matlib.ones(3).T)
			self.contacts[i].setKd(2.0 * np.sqrt(kp_contact) * matlib.ones(3).T)
			H_ref = self.robot.framePosition(self.data, self.model.getFrameId(name))
			self.contacts[i].setReference(H_ref)
			self.contacts[i].useLocalFrame(False)
			self.invdyn.addRigidContact(self.contacts[i], w_forceRef, 1.0, 1)
			
		
		## TSID Trajectory 
		
		# POSTURE Task
		pin.loadReferenceConfigurations(self.model, srdf, False)
		
		self.q_ref = self.model.referenceConfigurations['straight_standing'] 
		self.trajPosture = tsid.TrajectoryEuclidianConstant("traj_joint", self.q_ref[7:])
		# Set the trajectory as reference for the posture task
		self.samplePosture = self.trajPosture.computeNext()
		self.postureTask.setReference(self.samplePosture)
		
		# LOCK Task 
		# The mask is enough to set the shoulder acceleration to 0 because 0 is the initial configuration for the shoulders
		trajLock = tsid.TrajectoryEuclidianConstant("traj_lock_shoulder", self.q_ref[7:])

		sampleLock = trajLock.computeNext()
		self.lockTask.setReference(sampleLock)
		
		self.q0_FR_KFE = self.q_ref[12].copy()	# configuration value of the FR_KFE joint
		
		## Initialization of the solver

		# Use EiquadprogFast solver
		self.solver = tsid.SolverHQuadProgFast("qp solver")
		# Resize the solver to fit the number of variables, equality and inequality constraints
		self.solver.resize(self.invdyn.nVar, self.invdyn.nEq, self.invdyn.nIn)

				
	####################################################################
	#                      Torque Control method                       #
	####################################################################
	def control(self, qmes12, vmes12, t):
		
		"""if (t>3.0):
			self.q_ref[12] = 0.2 * np.sin(self.omega*t) + self.q0_FR_KFE
			
			self.trajPosture = tsid.TrajectoryEuclidianConstant("traj_joint", self.q_ref[7:])
			
			self.samplePosture = self.trajPosture.computeNext()
			self.postureTask.setReference(self.samplePosture)
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
		torques12 = P * (self.qdes[7:] - qmes12[7:]) + D * (self.vdes[6:] - vmes12[6:]) + tau_ff
		
		torques8 = np.concatenate((torques12[1:3], torques12[4:6], torques12[7:9], torques12[10:12]))
		
		# Saturation to limit the maximal torque
		t_max = 2.5
		tau = np.maximum(np.minimum(torques8, t_max * np.ones((8,1))), -t_max * np.ones((8,1)))
		
		self.error = self.error or (self.sol.status!=0) or (qmes12[8] < -np.pi/2) or (qmes12[11] < -np.pi/2) or (qmes12[14] < -np.pi/2) or (qmes12[17] < -np.pi/2) or (qmes12[8] > np.pi/2) or (qmes12[11] > np.pi/2) or (qmes12[14] > np.pi/2) or (qmes12[17] > np.pi/2)
		
		return tau

# Parameters for the controller

dt = 0.001				# controller time step

q0 = np.zeros((19,1))	# initial configuration

omega = 1.0				# sinus pulsation
