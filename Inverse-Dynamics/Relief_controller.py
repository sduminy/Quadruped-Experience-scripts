# coding: utf8


########################################################################
#                                                                      #
#                    Control mode : tau = - D * v^                     #
#                                                                      #
########################################################################

import pinocchio as pin
import numpy as np 
import matplotlib.pylab as plt 

pin.switchToNumpyMatrix()


########################################################################
#                    Class for a Relief Controller                     #
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
		
		# D Torque controller,
		D = 0.2
		tau = -D * vmes
		
		# Saturation to limit the maximal torque
		t_max = 2.5
		tau = np.maximum(np.minimum(tau, t_max * np.ones((8,1))), -t_max * np.ones((8,1)))
		
		# Logging
		""" if (self.i<self.N_LOG):
			self.times[self.i,0] = computing_time
			for j in range(8):
				self.des_positions[self.i,j] = self.qdes[j]
				self.des_velocities[self.i,j] = self.vdes[j]
				self.meas_positions[self.i,j] = qmes[j]
				self.meas_velocities[self.i,j] = vmes[j]
		self.i += 1
 """
		return tau.flatten()

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

class controller_12dof:
	
	def __init__(self):
		self.error = False
	
	####################################################################
	#                      Torque Control method                       #
	####################################################################
	def control(self, qmes12, vmes12, t):
		
		# D Torque controller,
		D = 0.2
		torques12 = -D * vmes12[6:]
		
		torques8 = np.concatenate((torques12[1:3], torques12[4:6], torques12[7:9], torques12[10:12]))
		
		# Saturation to limit the maximal torque
		t_max = 2.5
		tau = np.maximum(np.minimum(torques8, t_max * np.ones((8,1))), -t_max * np.ones((8,1)))
		
		return tau
