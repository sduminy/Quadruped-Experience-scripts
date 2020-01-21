# coding: utf8


########################################################################
#                                                                      #
#              Control law : tau = P(q*-q^) - D(v^)                    #
#                                                                      #
########################################################################

import pinocchio as pin
import numpy as np 

pin.switchToNumpyMatrix()


########################################################################
#                Class for a proportionnal Controller                  #
########################################################################

class controller:
	
	def __init__(self, q0, omega, t, N_LOG=0):
		self.omega = omega
		self.q0 = q0
		self.qdes = q0.copy()
		self.vdes = np.zeros((8,1))
		self.ades = np.zeros((8,1))
		self.error = False
		self.i=0
		self.N_LOG = N_LOG
		if N_LOG >0:
			self.times=np.zeros((N_LOG,1))
		
	####################################################################
	#                      Torque Control method                       #
	####################################################################
	def control(self, qmes, vmes, t):
		# Definition of qdes, vdes and ades
		self.qdes = np.sin(self.omega * t) + self.q0
		self.vdes = self.omega * np.cos(self.omega * t)
		self.ades = -self.omega**2 * np.sin(self.omega * t)
		
		# PD Torque controller
		P = np.diag((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0))
		D = 0.1*np.diag((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
		tau = np.array(np.matrix(np.diag(P * (self.qdes - qmes) - D * vmes)).T)
		
		# Saturation to limit the maximal torque
		t_max = 1.
		tau = np.maximum(np.minimum(tau, t_max * np.ones((8,1))), -t_max * np.ones((8,1)))
		
		# Safety bounds ; call the safety controller if reached
		self.error = self.error or (qmes[0] < -np.pi/2) or (qmes[2] < -np.pi/2) or (qmes[4] < -np.pi/2) or (qmes[6] < -np.pi/2) or (qmes[0] > np.pi/2) or (qmes[2] > np.pi/2) or (qmes[4] > np.pi/2) or (qmes[6] > np.pi/2)

		#Logging
		if (self.i<self.N_LOG):
			self.times[i,0] = t
		self.i+=1

		return tau.flatten()

# Parameters for the controller

dt = 0.001				# controller time step

omega = np.zeros((8,1))		# sinus pulsation

q0 = np.zeros((8,1))

for i in range(8):
	omega[i] = 1.0
	q0[i] = i/20
