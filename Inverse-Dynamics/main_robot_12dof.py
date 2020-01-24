# coding: utf8

import time
import numpy as np 
import libmaster_board_sdk_pywrap as mbs

# import the controller class with its parameters
from P_controller import controller, dt, q0, omega
import Safety_controller
import EmergencyStop_controller


def example_script(name_interface):
	
    ########################################################################
    #                Parameters definition & Initialization                #
    ########################################################################

    # Simulation parameters
    N_SIMULATION = 30000	# number of time steps simulated
    t = 0.0  				# time
    # Initialize the error for the simulation time
    time_error = False
    
    N_SLAVES = 6  # Maximum number of controled drivers
	N_SLAVES_CONTROLED = 2  # Current number of controled drivers

	iq_sat = 1.0  # Maximum amperage (A)
	init_pos = [0.0 for i in range(N_SLAVES * 2)]  # List that will store the initial position of motors
	state = 0  # State of the system (ready (1) or not (0))

	robot_if = mbs.MasterBoardInterface(name_interface) # Robot interface
	robot_if.Init()  # Initialization of the interface between the computer and the master board
	
	for i in range(N_SLAVES_CONTROLED):  # We enable each controler driver and its two associated motors
		robot_if.GetDriver(i).motor1.SetCurrentReference(0)
		robot_if.GetDriver(i).motor2.SetCurrentReference(0)
		robot_if.GetDriver(i).motor1.Enable()
		robot_if.GetDriver(i).motor2.Enable()
		robot_if.GetDriver(i).EnablePositionRolloverError()
		robot_if.GetDriver(i).SetTimeout(5)
		robot_if.GetDriver(i).Enable()

	qmes8 = np.zeros((8,1))
	vmes8 = np.zeros((8,1))
	K = 1.0/(9*0.02)    # proportionnal gain to convert joint torques to current : tau_mot = K'*I ; tau_joints = Kred*K'*I ; K' = 0.02 ; Kred = 9
    
    print("-- Start of example script --")

    ########################################################################
    #                            Control loop                              #
    ########################################################################

    myController = controller(q0, omega, t)
    mySafetyController = Safety_controller.controller()
    myEmergencyStop = EmergencyStop_controller.controller()


    for i in range (N_SIMULATION):
        
        time_start = time.time()
        
        robot_if.ParseSensorData()  # Read sensor data sent by the masterboard

        if (state == 0):  # If the system is not ready
            state = 1
            for i in range(N_SLAVES_CONTROLED * 2):  # Check if all motors are enabled and ready
                if not (robot_if.GetMotor(i).IsEnabled() and robot_if.GetMotor(i).IsReady()):
                    state = 0
                init_pos[i] = robot_if.GetMotor(i).GetPosition()
                t = 0.0
        else:  # If the system is ready
            for i in range(N_SLAVES_CONTROLED * 2):
                if robot_if.GetMotor(i).IsEnabled():
                    qmes8[i] = robot_if.GetMotor(i).GetPosition()
                    vmes8[i] = robot_if.GetMotor(i).GetVelocity()
			
			# Conversion (from 8 to 12 DOF) for TSID computation
            # Base velocity is not measurable but maybe 
            # we can get position and orientation of the base 
            # with robot_if.GetIMU() 
			qmes12 = np.concatenate((np.zeros((7,1)), np.matrix([0.]), qmes8[7:9], np.matrix([0.]), qmes8[9:11], np.matrix([0.]), qmes8[11:13], np.matrix([0.]), qmes8[13:15]))
			vmes12 = np.concatenate((np.zeros((7,1)), np.matrix([0.]), vmes8[6:8], np.matrix([0.]), vmes8[8:10], np.matrix([0.]), vmes8[10:12], np.matrix([0.]), vmes8[12:14]))
			

        ####################################################################
        #                Select the appropriate controller 				   #
        #								&								   #
        #				Load the joint torques into the robot			   #
        ####################################################################
        
        # If the limit bounds are reached, controller is switched to a pure derivative controller
        if(myController.error):
            print("Safety bounds reached. Switch to a safety controller")
            myController = mySafetyController
            
        # If the simulation time is too long, controller is switched to a zero torques controller
        time_error = time_error or (time.time()-time_start > 0.003)
        if (time_error):
            print("Computation time lasted to long. Switch to a zero torque control")
            myController = myEmergencyStop
            
        # Retrieve the joint torques from the appropriate controller
        jointTorques = myController.control(qmes12, vmes12, t)
        
        # Time incrementation
        t += dt
        
        if (state==1):
            for i in range(N_SLAVES_CONTROLED * 2):
                if robot_if.GetMotor(i).IsEnabled():
                    cur = K * jointTorques[i,0]
                    if (cur > iq_sat):  # Check saturation
                        cur = iq_sat
                    if (cur < -iq_sat):
                        cur = -iq_sat
                    robot_if.GetMotor(i).SetCurrentReference(cur)  # Set reference currents
        robot_if.SendCommand()  # Send the reference currents to the master board
    
    # At the end of the loop, set the reference current to zero
    for i in range(N_SLAVES_CONTROLED * 2):
        if robot_if.GetMotor(i).IsEnabled():
            robot_if.GetMotor(i).SetCurrentReference(0.0)  # Set reference currents
    robot_if.SendCommand()

    robot_if.Stop()  # Shut down the interface between the computer and the master board
    
    print("-- End of example script --")


def main(argv):
    name_interface = ""  # Name of the interface (use ifconfig in a terminal), for instance "enp1s0"
    try:
        opts, args = getopt.getopt(argv, "hi:")
    except getopt.GetoptError:
        print("example.py -i <interface>")
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print("example.py -i <interface>")
            sys.exit()
        elif opt in ("-i") and isinstance(arg, str):
            name_interface = arg
    print("name_interface: ", name_interface)
    example_script(name_interface)


if __name__ == "__main__":
    main(sys.argv[1:])