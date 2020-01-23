# coding: utf8


########################################################################
#                                                                      #
#          Control law : tau = P(q*-q^) + D(v*-v^) + tau_ff            #
#                                                                      #
########################################################################

import pinocchio as pin
import numpy as np
import tsid

pin.switchToNumpyMatrix()


########################################################################
#            Class for a PD with feed-forward Controller               #
########################################################################

class controller:
    
    def __init__(self, q0, omega, t):
        self.omega = omega
        self.q0 = q0
        self.qdes = q0.copy()
        self.vdes = np.zeros((8,1))
        self.ades = np.zeros((8,1))
        self.error = False
        
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

        robot = tsid.RobotWrapper(urdf, vector, False)
        
        self.model = robot.model()
        self.data = robot.data()
        
    ####################################################################
    #                      Torque Control method                       #
    ####################################################################
    def control(self, qmes, vmes, t):
        # Definition of qdes, vdes and ades
        self.qdes = 0.4*np.sin(self.omega * t) + self.q0
        self.vdes = 0.4*self.omega * np.cos(self.omega * t)
        self.ades = -0.4*self.omega**2 * np.sin(self.omega * t)
        
        tau_ff = pin.rnea(self.model, self.data, self.qdes, self.vdes, self.ades)		# feed-forward torques
        
        # PD Torque controller
        P = 5*np.diag((1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        D = 0.05*np.diag((1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        K_ff = np.diag((0.8, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        
        if (t>7.5):
            K_ff = - np.diag((0.8, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        
        tau = np.array(P @ (self.qdes - qmes) + D @ (self.vdes - vmes) + K_ff @ tau_ff)
        
        # Saturation to limit the maximal torque
        t_max = 1.0
        tau = np.maximum(np.minimum(tau, t_max * np.ones((8,1))), -t_max * np.ones((8,1)))
        
        self.error = self.error or (qmes[0] < -np.pi/2) or (qmes[2] < -np.pi/2) or (qmes[4] < -np.pi/2) or (qmes[6] < -np.pi/2) or (qmes[0] > np.pi/2) or (qmes[2] > np.pi/2) or (qmes[4] > np.pi/2) or (qmes[6] > np.pi/2)
        
        return tau.flatten()

# Parameters for the controller

omega = np.zeros((8,1))		# sinus pulsation

q0 = np.zeros((8,1))		# initial configuration

for i in range(8):
    omega[i] = 5.0
