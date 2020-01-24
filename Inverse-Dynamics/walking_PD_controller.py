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
    
    def __init__(self):
        
        self.error = False

        self.q = np.load('reference_q_v.npz')['qdes']
        self.v = np.load('reference_q_v.npz')['vdes']
        self.N_POINTS = len(self.q)
        self.iter = 0

    ####################################################################
    #                      Torque Control method                       #
    ####################################################################
    def control(self, qmes, vmes):
        # Definition of qdes and vdes 
        
        self.qdes = self.q[self.iter % self.N_POINTS]
        self.vdes = self.v[self.iter % self.N_POINTS]
        
        # PD Torque controller
        P_final = 5.
        D_final = 0.05

        #slowly increase the P and D
        if self.iter < 3000:
            P = self.iter*P_final/3000.0
            D = self.iter*D_final/3000.0
        else:
            P = P_final
            D = D_final
        
        tau = np.array(P * (self.qdes - qmes) + D * (self.vdes - vmes))
        
        # Saturation to limit the maximal torque
        t_max = 1.0
        tau = np.maximum(np.minimum(tau, t_max * np.ones((8,1))), -t_max * np.ones((8,1)))
        
        self.error = self.error or (qmes[0] < -np.pi/2) or (qmes[2] < -np.pi/2) or (qmes[4] < -np.pi/2) or (qmes[6] < -np.pi/2) or (qmes[0] > np.pi/2) or (qmes[2] > np.pi/2) or (qmes[4] > np.pi/2) or (qmes[6] > np.pi/2)
        
        self.iter += 1

        return tau.flatten()

