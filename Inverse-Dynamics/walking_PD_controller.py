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

        self.q = np.load('reference_q_v_slow.npz')['qdes']       # load the reference position and velocity vectors
        self.v = np.load('reference_q_v_slow.npz')['vdes']       # the shape of each vector is (8,)
        self.N_POINTS = len(self.q)                         # N_POINTS is the number of points needed to complete the trajectory
        self.iter = 0                                       # iter permits to slowly increase the gains

    ####################################################################
    #                      Torque Control method                       #
    ####################################################################
    def control(self, qmes, vmes):
        
        # Definition of qdes and vdes 
        self.qdes = np.array([ self.q[self.iter % self.N_POINTS] ]).T       # give the reference values to qdes and vdes  
        self.vdes = np.array([ self.v[self.iter % self.N_POINTS] ]).T       # np.array([ ]).T transforms the shape to (8,1)
         
    
        # PD Torque controller
        P_final = 3.0        # final proportionnal gain
        D_final = 0.07      # final derivative gain
        
        # Slowly increase the P and D
        
        if self.iter < 10000:
            P = self.iter*P_final/10000.0
            D = self.iter*D_final/10000.0
        else:
            P = P_final
            D = D_final
        
        tau = np.array(P * (self.qdes - qmes) + D * (self.vdes - vmes))
        
        # Saturation to limit the maximal torque
        t_max = 1.0
        tau = np.maximum(np.minimum(tau, t_max * np.ones((8,1))), -t_max * np.ones((8,1)))
        
        # Error defines the position bounds of the robot (hips should not go over +/- 90°)
        self.error = self.error or (qmes[0] < -np.pi/2) or (qmes[2] < -np.pi/2) or (qmes[4] < -np.pi/2) or (qmes[6] < -np.pi/2) or (qmes[0] > np.pi/2) or (qmes[2] > np.pi/2) or (qmes[4] > np.pi/2) or (qmes[6] > np.pi/2)
        
        self.iter += 1

        return tau.flatten()    # reduces the torques vector to 1 dimension : shape (8,)

