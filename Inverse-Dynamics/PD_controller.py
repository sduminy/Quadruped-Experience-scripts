# coding: utf8


########################################################################
#                                                                      #
#              Control law : tau = P(q*-q^) + D(v*-v^)                 #
#                                                                      #
########################################################################

import pinocchio as pin
import numpy as np 

pin.switchToNumpyMatrix()


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
		self.qdes = np.sin(self.omega * t) + self.q0
		self.vdes = self.omega * np.cos(self.omega * t)
		self.ades = -self.omega**2 * np.sin(self.omega * t)
		
		# PD Torque controller
		P = np.diag((5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
		D = 0.1*np.diag((3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
		tau = np.array(np.matrix(np.diag(P * (self.qdes - qmes) + D * (self.vdes - vmes))).T)
		
		# Saturation to limit the maximal torque
		t_max = 2.5
		tau = np.maximum(np.minimum(tau, t_max * np.ones((8,1))), -t_max * np.ones((8,1)))
		
		self.error = self.error or (qmes[0] < -np.pi/2) or (qmes[2] < -np.pi/2) or (qmes[4] < -np.pi/2) or (qmes[6] < -np.pi/2) or (qmes[0] > np.pi/2) or (qmes[2] > np.pi/2) or (qmes[4] > np.pi/2) or (qmes[6] > np.pi/2)
	
		return tau.flatten()

# Parameters for the controller

dt = 0.001				# controller time step

omega = np.zeros((8,1))		# sinus pulsation

q0 = np.zeros((8,1))

for i in range(8):
	omega[i] = 1.0
	q0[i] = i/20
