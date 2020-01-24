# coding: utf8

from time import clock
import time
import sys
import getopt
import os

import libmaster_board_sdk_pywrap as mbs
import numpy as np

# import the controller class with its parameters
from walking_PD_controller import controller
import log_class
import Safety_controller
import EmergencyStop_controller
from masterboard_utils import *

dt = 0.001  #  Time step

def example_script(name_interface):

    ########################################################################
    #                Parameters definition & Initialization                #
    ########################################################################

    t = 0  				# time
    cpt = 0             # iteration counter
    time_error = False  # Initialize the error for the simulation time
    state = 0  # State of the system (ready (1) or not (0))
    
    os.nice(-20)    # Set the highest priority to process the script (~Real-Time)
    
    robot_if = mbs.MasterBoardInterface(name_interface)  # Robot interface
    robot_if.Init()  # Initialization of the interface between the computer and the master board

    enable_all_motors(robot_if)
 
    qmes = np.zeros((8, 1)) # intialize the measured positions vector
    vmes = np.zeros((8, 1)) # intialize the measured velocities vector
    
    # Proportionnal gain to convert joint torques to current : tau_mot = K'*I 
    # tau_joints = Kred*K'*I ; K' = 0.02 Nm/A ; Kred = 9.0
    Kt = 1.0/(9*0.02)   # A/Nm

    flag_1st_it = True      # to intialize the filtered velocities vector
    alpha = 0.025            # filter coefficient


    print("-- Start of example script --")

    ########################################################################
    #                            Control loop                              #
    ########################################################################

    # Initialize the main controller
    myController = controller(q0, omega, t)
    myLog = log_class.log(N_LOG=14000)

    last = clock()

    while ((not robot_if.IsTimeout()) and (clock() < 20)):  # Stop after 15 seconds (around 5 seconds are used at the start for calibration)
        if ((clock() - last) > dt):
            last = clock()
            cpt += 1
            
            # Time incrementation
            t += dt
            
            robot_if.ParseSensorData()  # Read sensor data sent by the masterboard
            
            if (state == 0):  #  If the system is not ready
                t = 0
                if (are_all_motor_ready(robot_if)):
                    state = 1 


            if (state==1):  # If the system is ready
                
                # Get measured positions and velocities
                update_measured_q_v(robot_if, qmes, vmes)
                
                if (flag_1st_it):   # initialize the filtered velocities vector
                    vfilt = vmes
                    flag_1st_it = False

                vfilt = vmes * alpha + vfilt * (1 - alpha)

                ####################################################################
                #                Select the appropriate controller 				   #
                #								&								   #
                #				Load the joint torques into the robot			   #
                ####################################################################

                # If the limit bounds are reached, controller is switched to a pure derivative controller
                if(myController.error):
                    print("Safety bounds reached. Switch to a safety controller")
                    mySafetyController = Safety_controller.controller(myController.qdes, myController.vdes)
                    myController = mySafetyController

                # If the simulation time is too long, controller is switched to a zero torques controller
                time_error = time_error or ((clock()-last) > 0.003)
                if (time_error):
                    print("Computation time lasted to long. Switch to a zero torque control")
                    myEmergencyStop = EmergencyStop_controller.controller(myController.qdes, myController.vdes)
                    myController = myEmergencyStop

                # Retrieve the joint torques from the appropriate controller
                jointTorques = myController.control(qmes, vmes, t)
                # Set the desired torques to the motors
                set_desired_torques(robot_if, jointTorques)

                # Logging
                myLog.log_method(clock()-last, myController.qdes, myController.vdes, qmes, vmes, vfilt, jointTorques)
                
            if ((cpt % 100) == 0):  # Display state of the system once every 100 iterations of the main loop
                print(chr(27) + "[2J")
                # To read IMU data in Python use robot_if.imu_data_accelerometer(i), robot_if.imu_data_gyroscope(i)
                # or robot_if.imu_data_attitude(i) with i = 0, 1 or 2
                robot_if.PrintIMU()
                robot_if.PrintADC()
                robot_if.PrintMotors()
                robot_if.PrintMotorDrivers()
                sys.stdout.flush()  # for Python 2, use print( .... , flush=True) for Python 3
            robot_if.SendCommand()  # Send the reference currents to the master board

    print("-- Shutting down --")
    
    np.savez('/home/ada/Desktop/Script_Segolene_XP/Quadruped-Experience-scripts/Inverse-Dynamics/logs', times=myLog.times, des_positions=myLog.des_positions, des_velocities=myLog.des_velocities, meas_positions=myLog.meas_positions, meas_velocities=myLog.meas_velocities)
    
    myLog.plot_logs_velocities()

    robot_if.Stop()  # Shut down the interface between the computer and the master board
    
    if robot_if.IsTimeout():
        print("Masterboard timeout detected. Either the masterboard has been shut down or there has been a connection issue with the cable/wifi.")

    print("-- End of example script --")

    


def main(argv):
    # Name of the interface (use ifconfig in a terminal), for instance "enp1s0"
    name_interface=""
    try:
        opts, args=getopt.getopt(argv, "hi:")
    except getopt.GetoptError:
        print("example.py -i <interface>")
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print("example.py -i <interface>")
            sys.exit()
        elif opt in ("-i") and isinstance(arg, str):
            name_interface=arg
    print("name_interface: ", name_interface)
    example_script(name_interface)


if __name__ == "__main__":
    main(sys.argv[1:])
