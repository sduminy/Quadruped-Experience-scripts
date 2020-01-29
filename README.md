# Quadruped-Experience-scripts
Python scripts to be implemented on the ODRI robot, inspired by script from the repositories sduminy/Inverse-Kinematics and sduminy/TSID

## Description of the files
### Inverse-Kinematics
All the files in this folder aims to create a simulation of walking with the robot.
- `initialization_simulation.py` initialize the robot model (using `robots_loader.py`) and PyBullet simulator
- `walking_controller.py` contains controller functions for the walk (the first one `c_walking_IK()` with a sinusoidal curves trajectory of the feet, the second one `c_walking_IK_bezier()` with Bezier curves as trajectory during the swing phase). Those controllers are based on Inverse-Kinematics : it computes the Jacobian of the errror to find the velocity which mminimizes the error. 
- `PD.py` is the PD torques controller. The controllers' output are torques, which are sent to the robot via PyBullet.
- `main.py` computes the simulation in a for-loop, creates 2 numpy array variables `Q` and `V` to log the reference configurations and velocities and save them into a file `reference_q_v.npz`. They will serve as reference for the PD controller tested on the robot `Inverse-Dynamics\walking_PD_controller.py`.

### Inverse-Dynamics
The files in this folder use Inverse-Dynamics to simulate/experience some movements on the robot.
#### Simulation
- `main_simu.py` serves to test the different controllers on simulation first using PyBullet and the same environment as the one in **Inverse-Kinematics** 
- `main_simu_12dof.py` is for a simulation using TSID and contact forces, which force to use the 12 dof model of the robot
- `main_simu_walk.py` was specifically design to test the walk in simulation (using the PD controller with the references logged in the `reference_q_v_{slow/fast}.npz` files 
### Experimentation
- `example.py` is the example to test if the masterboard interface works well
- `masterboard_utils.py` contains all functions from the masterboard (initialization of motors, saturation of the current, desired torques setting). Caution : the indexes of the configuration joints are not the same as the one used for the motors controlled by the masterboard. This file fix this problem.
- `libmaster-board-sdk.so` and its Pyhton bindings `libmaster-board-sdk_pywrap.so` is the library of the masterboard
- `main_robot.py` initializes the masterboard interface, calls a controller and computes the experimentation ; it also logs some interesting values into a `logs.npz` file using a class `log_class.py`. Those values can be displayed using `plot_logs.py`.
- `main_robot_12dof.py` would be the same for a 12dof robot but it has not been tested so must need some adjustments
- `main_robot_walk.py` is the main programm specifically designed for the experience of walking with the `walking_PD_controller.py`
### Controllers
- `P_controller.py`, `PD_controller.py` and `PDff_controller` are simple controllers to test first on the robot. They set a sinusoidal reference joint position to one or multiple joints. Those files were used to adjust the gains of the proportional and derivative terms.
- `Safety_controller.py` is a pure derivative controller which is switched on when position bounds are reached.
- `EmergencyStop_controller.py` is a zero-torques controller which is switched on when computation time is longer than 3 ms.
- `calibration_controller.py` was used to calculate the real torques exerced by one joint (using a scale and the moment arm) to recover the relation between currents and torques `torques_joints = Kred * torques_mot = Kred * Ki * currents_mot` to see if it was correct.
- `walking_PD_controller.py` is the controller used to experiment the walk on the robot by loading the reference joints positions and velocities with the `reference_q_v_small.npz` (for a trajectory period of 0.5 s) or `reference_q_v_fast.npz` (for a trajectory period of 0.3 s) files.
- `TSID_posture_controller.py` uses TSID Joint Posture task (from all joints to zero radian, to the reference configuration *straight_standing*) ; was tested on the robot
- `TSID_foot_placement_controller.py` uses TSID SE3Equality Task to set a foot to a desired position ; this has not been tested on the robot.
- `TSID_contacts_and_posture_controller.py` and `TSID_contacts_posture_and_com_controller.py` were not tested on the robot. Those controller requires to load the 12 dof robot model to work.	
