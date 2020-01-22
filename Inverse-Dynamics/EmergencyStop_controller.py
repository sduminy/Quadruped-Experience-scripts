# coding: utf8


########################################################################
#                                                                      #
#                      Control law : tau = 0                           #
#                                                                      #
########################################################################

import pinocchio as pin
import numpy as np 
import matplotlib.pylab as plt 

pin.switchToNumpyMatrix()


########################################################################
#              Class for an Emergency Stop Controller                  #
########################################################################

class controller:
	
	def __init__(self, qdes, vdes, times, i, des_pos, des_vel, meas_pos, meas_vel, N_LOG=0):
		self.error = False
		self.qdes = qdes
		self.vdes = vdes
		self.times = times
		self.i = i
		self.N_LOG = N_LOG
		self.des_positions = des_pos
		self.des_velocities = des_vel
		self.meas_positions = meas_pos
		self.meas_velocities = meas_vel
		
	####################################################################
	#                      Torque Control method                       #
	####################################################################
	def control(self, qmes, vmes, t, computing_time):
		
		# Torque controller
		tau = np.zeros((8,1))

		# Logging
		""" if (self.i<self.N_LOG):
			self.times[self.i,0] = computing_time
			for j in range(8):
				self.des_positions[self.i,j] = self.qdes[j]
				self.des_velocities[self.i,j] = self.vdes[j]
				self.meas_positions[self.i,j] = qmes[j]
				self.meas_velocities[self.i,j] = vmes[j]
		self.i += 1 """
		
		return tau

	####################################################################
	#                         Logs plot method                         #
	####################################################################
	def plot_logs(self):
		plt.figure(1)
		plt.plot(self.times, 'k+')
		plt.grid()
		plt.title('Computing time')
		plt.show()

		plt.figure(2)
		for k in range(8):
			plt.subplot(4,2,k+1)
			plt.plot(self.des_positions[:,k], label='Desired positions')
			plt.plot(self.meas_positions[:,k], label='Measured positions')
		plt.legend()
		plt.title('Positions tracking')
		plt.show()
		
		plt.figure(3)
		for k in range(8):
			plt.subplot(4,2,k+1)
			plt.plot(self.des_velocities[:,k], label='Desired velocities')
			plt.plot(self.meas_velocities[:,k], label='Measured velocities')
		plt.legend()
		plt.title('Velocities tracking')
		plt.show()