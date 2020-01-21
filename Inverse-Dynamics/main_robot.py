# coding: utf8
from time import clock
import time
import sys
import getopt
import os

import libmaster_board_sdk_pywrap as mbs
import numpy as np
# import the controller class with its parameters
from P_controller import controller, q0, omega
import Relief_controller
import EmergencyStop_controller
from masterboard_utils import *
dt = 0.001  #  Time step

def example_script(name_interface):

    ########################################################################
    #                Parameters definition & Initialization                #
    ########################################################################


    t = 0  				# time
    cpt = 0       
    # Initialize the error for the simulation time
    time_error = False
    state = 0  # State of the system (ready (1) or not (0))
    
    os.nice(-20)
    robot_if = mbs.MasterBoardInterface(name_interface)  # Robot interface
    robot_if.Init()  # Initialization of the interface between the computer and the master board

    #  We enable each controler driver and its two associated motors
    for i in range(N_SLAVES_CONTROLED):
        robot_if.GetDriver(i).motor1.SetCurrentReference(0)
        robot_if.GetDriver(i).motor2.SetCurrentReference(0)
        robot_if.GetDriver(i).motor1.Enable()
        robot_if.GetDriver(i).motor2.Enable()
        robot_if.GetDriver(i).EnablePositionRolloverError()
        robot_if.GetDriver(i).SetTimeout(5)
        robot_if.GetDriver(i).Enable()

    qmes = np.zeros((8, 1))
    vmes = np.zeros((8, 1))
    # A/Nm proportionnal gain to convert joint torques to current : tau_mot = K'*I ; tau_joints = Kred*K'*I ; K' = 0.02 ; Kred = 9
    Kt = 1.0/(9*0.02)

    print("-- Start of example script --")

    ########################################################################
    #                            Control loop                              #
    ########################################################################

    myController = controller(q0, omega, t, N_LOG = 10)
    myReliefController = Relief_controller.controller()
    myEmergencyStop = EmergencyStop_controller.controller()
    
    last = clock()

    while ((not robot_if.IsTimeout()) and (clock() < 60)):  # Stop after 15 seconds (around 5 seconds are used at the start for calibration)
        if ((clock() - last) > dt):
            last = clock()
            cpt += 1
            
            # Time incrementation
            t += dt
            
            time_start = time.time()
            robot_if.ParseSensorData()  # Read sensor data sent by the masterboard
            
            if (state == 0):  #  If the system is not ready
                t = 0
                if (are_all_motor_ready(robot_if)):
                    state =1 
    

            if(state==1):  # If the system is ready
                #Get measured positions and velocities
                update_measured_q_v(robot_if,qmes,vmes)

                ####################################################################
                #                Select the appropriate controller 				   #
                #								&								   #
                #				Load the joint torques into the robot			   #
                ####################################################################

                # If the limit bounds are reached, controller is switched to a pure derivative controller
                if(myController.error):
                    print("Safety bounds reached. Switch to a safety controller")
                    myController = myReliefController

                # If the simulation time is too long, controller is switched to a zero torques controller
                time_error = time_error or (time.time()-time_start > 0.003)
                if (time_error):
                    print("Computation time lasted to long. Switch to a zero torque control")
                    myController = myEmergencyStop

                # Retrieve the joint torques from the appropriate controller
                jointTorques = myController.control(qmes, vmes, t)
                set_desired_torques(robot_if,jointTorques)
                
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

    print("-- end of the loop --")

    print("-- shutingdown... --")
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
