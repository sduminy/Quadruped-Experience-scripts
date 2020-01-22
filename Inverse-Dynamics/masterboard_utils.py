N_SLAVES_CONTROLED = 4  # Current number of controled drivers
Kt = 1.0/(9*0.02)
iq_sat = 0.8  # Maximum amperage (A)

def enable_all_motors(robot_if):
    # Enable each controler driver and its two associated motors
    for i in range(N_SLAVES_CONTROLED):
        robot_if.GetDriver(i).motor1.SetCurrentReference(0)
        robot_if.GetDriver(i).motor2.SetCurrentReference(0)
        robot_if.GetDriver(i).motor1.Enable()
        robot_if.GetDriver(i).motor2.Enable()
        robot_if.GetDriver(i).EnablePositionRolloverError()
        robot_if.GetDriver(i).SetTimeout(5)
        robot_if.GetDriver(i).Enable()
    return

def saturate(value, saturation):
    if (value > saturation):  #  Check saturation
        value = saturation
    if (value < -saturation):
        value = -saturation
    return value


def set_desired_torques(robot_if, tau):
    Kt = 1.0/(9*0.02)
    for i in range(N_SLAVES_CONTROLED * 2):
                    cur = 0.0
                    if robot_if.GetMotor(i).IsEnabled():
                        if (i == 0):
                            # left front hip
                            cur = -saturate(Kt * tau[0], iq_sat)
                        if (i == 1):
                            # left front knee
                            cur = -saturate(Kt * tau[1], iq_sat)
                        if (i == 2):
                            # right front knee
                            cur = +saturate(Kt * tau[3], iq_sat)
                        if (i == 3):
                            # right front hip
                            cur = +saturate(Kt * tau[2], iq_sat)
                        if (i == 4):
                            # left hint knee
                            cur = -saturate(Kt * tau[5], iq_sat)
                        if (i == 5):
                            # left hint hip
                            cur = -saturate(Kt * tau[4], iq_sat)
                        if (i == 6):
                            # right hint hip
                            cur = +saturate(Kt * tau[6], iq_sat)
                        if (i == 7):
                            # right hint knee
                            cur = +saturate(Kt * tau[7], iq_sat)
                        robot_if.GetMotor(i).SetCurrentReference(cur)
    return


def update_measured_q_v(robot_if, q, v):
    for i in range(N_SLAVES_CONTROLED * 2):
        if robot_if.GetMotor(i).IsEnabled():
            if (i == 0):
                # left front hip
                q[0] = -robot_if.GetMotor(i).GetPosition()/9.0
                v[0] = -robot_if.GetMotor(i).GetVelocity()/9.0
            if (i == 1):
                # left front knee
                q[1] = -robot_if.GetMotor(i).GetPosition()/9.0
                v[1] = -robot_if.GetMotor(i).GetVelocity()/9.0
            if (i == 2):
                # right front knee
                q[3] = +robot_if.GetMotor(i).GetPosition()/9.0
                v[3] = +robot_if.GetMotor(i).GetVelocity()/9.0
            if (i == 3):
                # right front hip
                q[2] = +robot_if.GetMotor(i).GetPosition()/9.0
                v[2] = +robot_if.GetMotor(i).GetVelocity()/9.0
            if (i == 4):
                # left hind knee
                q[5] = -robot_if.GetMotor(i).GetPosition()/9.0
                v[5] = -robot_if.GetMotor(i).GetVelocity()/9.0
            if (i == 5):
                # left hind hip
                q[4] = -robot_if.GetMotor(i).GetPosition()/9.0
                v[4] = -robot_if.GetMotor(i).GetVelocity()/9.0
            if (i == 6):
                # right hind hip
                q[6] = +robot_if.GetMotor(i).GetPosition()/9.0
                v[6] = +robot_if.GetMotor(i).GetVelocity()/9.0
            if (i == 7):
                # right hind knee
                q[7] = +robot_if.GetMotor(i).GetPosition()/9.0
                v[7] = +robot_if.GetMotor(i).GetVelocity()/9.0
            
    return

def are_all_motor_ready(robot_if):
    for i in range(N_SLAVES_CONTROLED * 2):
        if not (robot_if.GetMotor(i).IsEnabled() and robot_if.GetMotor(i).IsReady()):
            return False
    return True