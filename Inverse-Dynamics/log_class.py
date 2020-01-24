# coding: utf8


import numpy as np
import matplotlib.pylab as plt 



########################################################################
#           Class to log values from the controller/robot              #
########################################################################

class log:
    
    def __init__(self, N_LOG=0):
        
        self.N_LOG = N_LOG
        self.i = 0
        self.times = np.zeros((N_LOG,1))
        self.des_positions = np.zeros((N_LOG,8))
        self.des_velocities = np.zeros((N_LOG,8))
        self.meas_positions = np.zeros((N_LOG,8))
        self.meas_velocities = np.zeros((N_LOG,8))
        self.filt_velocities = np.zeros((N_LOG,8))
        self.torques = np.zeros((N_LOG, 8))
        self.cpt = np.zeros((N_LOG,1))
    
        
    ####################################################################
    #                            Log method                            #
    ####################################################################
    def log_method(self, computing_time, qdes, vdes, qmes, vmes, vfilt, tau, cpt):
        
        # Logging
        if (self.i < self.N_LOG):
            self.times[self.i,0] = computing_time
            self.cpt[self.i,0] = cpt
            for j in range(8):
                self.des_positions[self.i,j] = qdes[j]
                self.des_velocities[self.i,j] = vdes[j]
                self.meas_positions[self.i,j] = qmes[j]
                self.meas_velocities[self.i,j] = vmes[j]
                self.filt_velocities[self.i,j] = vfilt[j]
                self.torques[self.i,j] = tau[j]
        self.i += 1

    ####################################################################
    #                         Logs plot method                         #
    ####################################################################
    def plot_logs(self):
        plt.figure(1)
        plt.plot(self.cpt, label='temps')
        plt.legend()
        plt.show()

        """ plt.figure(1)
        plt.plot(self.times, 'k+')
        plt.grid()
        plt.title('Computing time')
        plt.show() """

        plt.figure(2)
        plt.suptitle('Positions tracking')
        for k in range(8):
            plt.subplot(4,2,k+1)
            plt.plot(self.des_positions[:,k], label='Desired positions')
            plt.plot(self.meas_positions[:,k], label='Measured positions')
        plt.legend()
        plt.show()
        
        plt.figure(3)
        plt.suptitle('Velocities tracking')
        for k in range(8):
            plt.subplot(4,2,k+1)
            plt.plot(self.des_velocities[:,k], label='Desired velocities')
            plt.plot(self.meas_velocities[:,k], label='Measured velocities')
        plt.legend()
        plt.show()

        plt.figure(4)
        plt.suptitle('Torques tracking')
        for k in range(8):
            plt.plot(self.torques[:,k])
        plt.show()


    def plot_logs_FL(self):
        
        plt.figure(1)
        plt.plot(self.times, 'k+')
        plt.grid()
        plt.title('Computing time')
        plt.show()

        plt.figure(2)
        plt.suptitle('Positions tracking')
        for k in range(2):
            plt.subplot(1,2,k+1)
            plt.plot(self.des_positions[:,k], label='Desired positions')
            plt.plot(self.meas_positions[:,k], label='Measured positions')
        plt.legend()
        plt.show()
        
        plt.figure(3)
        plt.suptitle('Velocities tracking')
        for k in range(2):
            plt.subplot(1,2,k+1)
            plt.plot(self.des_velocities[:,k], label='Desired velocities')
            plt.plot(self.meas_velocities[:,k], label='Measured velocities')
        plt.legend()
        plt.show()

    def plot_logs_velocities(self):
        
        plt.figure(1)
        plt.suptitle('Velocities tracking')
        plt.plot(self.des_velocities[:,0], label='Desired velocities')
        plt.plot(self.meas_velocities[:,0], '--r', label='Measured velocities')
        plt.plot(self.filt_velocities[:,0], 'g', label='Filtered velocities')   
        plt.legend()
        plt.show()

    def plot_logs_torques(self):
        
        plt.figure(1)
        plt.suptitle('Torques tracking')
        for k in range(8):
            plt.plot(self.torques[:,k])
        plt.show()