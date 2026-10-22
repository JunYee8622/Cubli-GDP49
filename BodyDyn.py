import numpy as np
import matplotlib.pyplot as plt
import random
import MotorDynamics as MD
import ThermalChecks as TC



usePlastic = True
if usePlastic:
    m_total = 0.32669
    x_dist = 2.61e-3
    y_dist = 108.12e-3
    m_wheel = 0.03582
    I_wheel = 75889.50e-9
    #I_wheel = 38127.85e-9 ADD I ROTOR
    #Ip =  4175714.14e-9 - I_wheel
    Ip = 5068286.99e-9 - I_wheel
else:
    m_total = 0.53079 # kg  total mass
    x_dist = 0.76e-3 # m  distance from pivot to cube centre of mass
    y_dist = 109.68e-3 # m  distance from pivot to cube centre of mass
    m_wheel = 0.23113 # kg  wheel mass - FROM CAD MODEL
    I_wheel =  245986.15e-9 #Taken from SOLDIWORKS 2D simplified model (Lzz)    ADD I ROTOR
    Ip =  8621584.76e-9 - I_wheel #Taken from SOLDIWORKS 2D simplified model (Izz) - removes wheel inertia as that is handled by T_wheel


g = 9.81 #Gravtational acceleration (m/s^2)
a_square = 0.15 # cube side length (m)






def angle_offset(x_dist, y_dist):
    angle_offset = np.arctan2(x_dist, y_dist)
    return(angle_offset)

def TotalMassLever(xdist, ydist, mass_total):
    distance = np.sqrt(ydist**2 + xdist**2)
    TotalMassLever = mass_total * distance
    return(TotalMassLever)


def calcI_Square(a, m): # The resistance of the cube to angular acceleration
    I = (2/3) * m * (a**2)
    return(I)

def calcI_Disk(r, m): # The resistance of the disk to angular acceleration
    I = (1/2) * m * (r**2)
    return(I)

def wheel_AAc(T, I): #angular acceleration of wheel
    return(T/I)

def L_w(a_square):  #Distance from pivot  to cube centre of mass
    L_w = a_square / np.sqrt(2) #Assumes rotational axis is at square centre
    return(L_w)


def I_p(I_square, m_wheel, L_w):    
    I_p = I_square + m_wheel*(L_w**2)
    return(I_p)

def body_AAc(T_g, T_d, T_w, I_p): # Body angular acc = torques / mass moment of inertia
    return((T_g + T_d - T_w)/I_p) # -ve wheel torque for Newtons 2L

def Grav_torque(TotalMassLever, theta, angledelta = 0):
    T_grav = (TotalMassLever) *g * np.sin(theta-angledelta) #Torque due to gravity on the cube
    return(T_grav)

def dist_torque(t = 0, use = False):
    if use == True:
        distMagnitude = 0.08
        n = 2
        #print("add disturbance torque functionality")
        '''t_impulse = [0.01, 0.2, 0.8, 4, 6 ] # seconds
        for t_i in t_impulse:
            if t == t_i:
                return(-0.1) # Nm'''
        if t % n < 0.5:
            return(distMagnitude)

    return(0)

####
#Control

def pid_control(theta, theta_vel, WheelV, cumulative_error, Kp = 5, Ki = 0.01, Kd = 0.1, Kp2 = 0.2, maxT = 0.110, use = True):
    if use == False:
        return(0)
    # Calculate the control torque using a PD controller
    P_control = Kp * theta  # Proportional term
    I_control = Ki * cumulative_error  # Integral term for wheel speed control
    D_control = Kd * theta_vel  # Derivative term
    P2_control = Kp2 * WheelV  # Proportional term for wheel speed
    T_control = P_control + I_control + D_control + P2_control
    # if abs(T_control) > maxT:
    #     T_control = np.sign(T_control) * maxT  # Limit the torque to the maximum motor torque
    return T_control



######
#Ip = I_p(calcI_Square(a_square, m_square), m_wheel, L_w(a_square))

#I_wheel = calcI_Disk(r_wheel, m_wheel)


m_leverTotal = TotalMassLever(x_dist, y_dist, m_total)





def simulate(sweep = False, simulate_time = 100, dt = 0.001, thetaInit_deg = 5, delayTime = 15e-3, Gains = (3.3, 0.001, 0.23, 0.001)):
    theta = np.deg2rad(thetaInit_deg) # initial angle (degrees)
    theta_vel = 0 # initial angular velocity (rad/s)
    time = 0
    WheelV = 0 # initial wheel velocity (rad/s)
    WheelVRelative = 0
    wheelAngle = 0
    Arr_angles = []
    Arr_timepoints = []
    WheelVels = []
    T_wheel = 0 # initial wheel torque
    trueAngleDelta = angle_offset(x_dist, y_dist)
    cumulative_error = 0 # For integral control of wheel speed
    saturated = False

    K = 36e-3
    max_motor_torque = 6 * K # Nm
    
    delayCycles = int(delayTime/dt) - 1

    if delayCycles <= 1:
        delayCycles = 1
    

    Arr_thetaMeasured = []
    Arr_thetaVelMeasured = []
    Arr_wheelVMeasured = []
    Arr_cumulativeError = []

    Arr_windingTemp = []
    Arr_housingTemp = []
    Arr_DTM = []
    Arr_Ilimit = []
    Arr_Idrawn = []
    
    Arr_wheelAngle = []

    for i in range(0, delayCycles):
        Arr_thetaMeasured.append(theta)
        Arr_thetaVelMeasured.append(theta_vel)
        Arr_wheelVMeasured.append(WheelV)
        Arr_cumulativeError.append(cumulative_error)

    thermILimit = 6 #A
    DTM = 0
    Twind = TC.celcius_to_kelvin(25) #winding initial temperature (cel)
    Thous = Twind
    Tamb = Twind
    while time < simulate_time and abs(np.rad2deg(theta)) < 45: # Stop if the cube falls over
        
        T_grav = Grav_torque(m_leverTotal, theta, trueAngleDelta)
        T_dist = dist_torque(time, use = True)
        #T_wheel = 0
        
        wheel_angular_acceleration = wheel_AAc(T_wheel, I_wheel)
        WheelV += wheel_angular_acceleration * dt # Update wheel velocity based on angular acceleration
        wheelAngle += (WheelV * dt) 
        
        


        body_angular_acceleration = body_AAc(T_grav, T_dist, T_wheel, Ip)
        theta_vel += body_angular_acceleration * dt # Update angular velocity based on angular acceleration
        theta += theta_vel * dt # Update angle based on angular velocity
        
        WheelVRelative = WheelV - theta_vel #relative velocity required for motor torque and wheel V measurements
        WheelVels.append(60*WheelVRelative/(2*np.pi)) #angular velocity in rpm

        
        Arr_timepoints.append(time)
        if saturated ==False:
            cumulative_error += WheelVRelative * dt # Update cumulative error for integral control - time multiplied in to allow for change in time step

        #angleErrorSum += abs(theta - trueAngleDelta)


        theta_measured = theta + random.uniform(np.deg2rad(-0.1), np.deg2rad(0.1)) # Add noise to the angle measurement
        theta_vel_measured = theta_vel + random.uniform(np.deg2rad(-1), np.deg2rad(1)) # Add noise to the angular velocity measurement
        WheelV_measured = WheelVRelative + random.uniform(np.deg2rad(-1), np.deg2rad(1)) # Add noise to the wheel velocity measurement

        Arr_thetaMeasured.append(theta_measured)
        Arr_thetaVelMeasured.append(theta_vel_measured)
        Arr_wheelVMeasured.append(WheelV_measured)
        Arr_cumulativeError.append(cumulative_error)

        Arr_angles.append(np.rad2deg(theta))
        Arr_wheelAngle.append(wheelAngle)
        
        T_demand = pid_control(Arr_thetaMeasured[-delayCycles], Arr_thetaVelMeasured[-delayCycles], Arr_wheelVMeasured[-delayCycles], Arr_cumulativeError[-delayCycles], Kp = Gains[0], Ki = Gains[1], Kd = Gains[2], Kp2 = Gains[3], maxT = max_motor_torque, use = True)
        T_friction = MD.MotorFrictionTorque(60*WheelVRelative/(2*np.pi), useTanh=(True, 5))
        T_wheel, saturated = MD.delivered_torque(60*WheelVRelative/(2*np.pi), T_demand/K, thermILimit, K)
        '''Thermal Current limits'''
        Idelivered = T_wheel / K
        thermILimit, Twind, Thous, DTM = TC.thermal_sim(Idelivered, Twind, Thous, Tamb, dt, DTM)

        Arr_Ilimit.append(thermILimit)
        Arr_windingTemp.append(Twind)
        Arr_housingTemp.append(Thous)
        Arr_DTM.append(DTM)
        Arr_Idrawn.append(Idelivered)
        ''''''
        
        T_wheel += T_friction
        time += dt # Update time
        if saturated and __name__ == "__main__":
            print("motor saturated")

    fig, ax = plt.subplots(1, 2, figsize=(10, 5))
    #ax[0].plot(Arr_timepoints, Arr_angles); ax[0].set_title('Cube Angle vs Time'); ax[0].set_xlabel('Time (s)'); ax[0].set_ylabel('Angle (degrees)')
    #ax[1].plot(Arr_timepoints, WheelVels); ax[1].set_title('Wheel Velocity vs Time'); ax[1].set_xlabel('Time (s)'); ax[1].set_ylabel('Wheel Velocity (rpm)')
    #print(WheelVels[-100:len(WheelVels)-1])
    plt.savefig("GUISprites/bodyDynamics.jpg")
    
    if __name__ == "__main__":
        ax[0].plot(Arr_timepoints, Arr_windingTemp)
        ax[1].plot(Arr_timepoints, Arr_Idrawn)
        print(f"trueAngleDelta: {np.rad2deg(trueAngleDelta)}")
        print(f"reading processing delayed by: {delayCycles} cycles ({delayCycles * dt } seconds)")
        plt.show()
        


    # if sweep:
    #     return(angleErrorSum, Arr_angles, WheelVels)
    return(Arr_timepoints, Arr_angles, Arr_wheelAngle, WheelVels)

if __name__ == "__main__":
    print(f"I_p: {Ip}")
    print(f"I_wheel: {I_wheel}")
    print(f"TotalMassLever: {m_leverTotal}")
    simulate()
    






'''
Things to do for simulation:
- Add arduino torque change delay + sensor read delay
- Filtering of sensor data
- 3D case incl gyroscopic effects
- Visulisation panel
- Other control methods (LQR, MPC, etc.)
- Replace I, m etc with data from CAD
'''