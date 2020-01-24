# coding: utf8

import numpy as np
import matplotlib.pylab as plt


times = np.load('logs.npz')['times']
des_positions = np.load('logs.npz')['des_positions']
des_velocities = np.load('logs.npz')['des_velocities']
meas_positions = np.load('logs.npz')['meas_positions']
meas_velocities = np.load('logs.npz')['meas_velocities']
""" torques = np.load('logs.npz')['torques']
iterations = np.load('logs.npz')['iter']
 """
""" plt.figure(0)
plt.plot(iterations, label='iterations')
plt.legend()
 """

plt.figure(1)
plt.plot(times, 'k+')
plt.grid()
plt.title('Computing time')


plt.figure(2)
plt.suptitle('Positions tracking')
for k in range(8):
    plt.subplot(4,2,k+1)
    plt.plot(des_positions[:,k], label='Desired positions')
    plt.plot(meas_positions[:,k], label='Measured positions')
plt.legend()


plt.figure(3)
plt.suptitle('Velocities tracking')
for k in range(8):
    plt.subplot(4,2,k+1)
    plt.plot(des_velocities[:,k], label='Desired velocities')
    plt.plot(meas_velocities[:,k], label='Measured velocities')
plt.legend()
plt.show()

""" plt.figure(4)
plt.suptitle('Torques tracking')
for k in range(8):
    plt.plot(torques[:,k])
plt.show() """