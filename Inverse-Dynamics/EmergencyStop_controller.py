# coding: utf8


########################################################################
#                                                                      #
#                      Control law : tau = 0                           #
#                                                                      #
########################################################################


import numpy as np 


########################################################################
#              Class for an Emergency Stop Controller                  #
########################################################################

class controller:
	
	def __init__(self, qdes, vdes):
		self.error = False
		self.qdes = qdes
		self.vdes = vdes
		
	####################################################################
	#                      Torque Control method                       #
	####################################################################
	def control(self, qmes, vmes, t):
		
		# Torque controller
		tau = np.zeros((8,1))
		
		return tau.flatten()