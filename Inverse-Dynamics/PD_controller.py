# coding: utf8


########################################################################
#                                                                      #
#              Control law : tau = P(q*-q^) + D(v*-v^)                 #
#                                                                      #
########################################################################


import numpy as np 



########################################################################
#                      Class for a PD Controller                       #
########################################################################

class controller:
	
	def __init__(self, q0, omega, t):
		self.omega = omega
		self.q0 = q0
		self.qdes = q0.copy()
		self.vdes = np.zeros((8,1))
		self.ades = np.zeros((8,1))
		self.error = False
		
	####################################################################
	#                      Torque Control method                       #
	####################################################################
	def control(self, qmes, vmes, t):
		# Definition of qdes, vdes and ades
		self.qdes = 0.4*np.sin(self.omega * t) + self.q0
		self.vdes = 0.4*self.omega * np.cos(self.omega * t)
		self.ades = -0.4*self.omega**2 * np.sin(self.omega * t)
		
		# PD Torque controller
		P = 5*np.diag((1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
		D = 0.05*np.diag((1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
		#tau = np.array(np.matrix(np.diag(P * (self.qdes - qmes) + D * (self.vdes - vmes))).T)
		tau = np.array(P @ (self.qdes - qmes) + D @ (self.vdes - vmes))
		
		# Saturation to limit the maximal torque
		t_max = 1.0
		tau = np.maximum(np.minimum(tau, t_max * np.ones((8,1))), -t_max * np.ones((8,1)))
		
		self.error = self.error or (qmes[0] < -np.pi/2) or (qmes[2] < -np.pi/2) or (qmes[4] < -np.pi/2) or (qmes[6] < -np.pi/2) or (qmes[0] > np.pi/2) or (qmes[2] > np.pi/2) or (qmes[4] > np.pi/2) or (qmes[6] > np.pi/2)
	
		return tau.flatten()

# Parameters for the controller

omega = np.zeros((8,1))		# sinus pulsation

q0 = np.zeros((8,1))

for i in range(8):
	omega[i] = 10.0
