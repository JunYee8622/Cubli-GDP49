import numpy as np
import matplotlib.pyplot as plt
import random
import tkinter as tk
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, 
NavigationToolbar2Tk)
import MotorDynamics as MD



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





max_motor_torque = 0.110 # Nm

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
        #print("add disturbance torque functionality")
        '''t_impulse = [0.01, 0.2, 0.8, 4, 6 ] # seconds
        for t_i in t_impulse:
            if t == t_i:
                return(-0.1) # Nm'''
        if t < 0.02:
            return(-0.1) # Nm
        else:
            return(0)
    else:
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
    if abs(T_control) > maxT:
        T_control = np.sign(T_control) * maxT  # Limit the torque to the maximum motor torque
    return T_control



######
#Ip = I_p(calcI_Square(a_square, m_square), m_wheel, L_w(a_square))

#I_wheel = calcI_Disk(r_wheel, m_wheel)


m_leverTotal = TotalMassLever(x_dist, y_dist, m_total)
print(f"I_p: {Ip}")
print(f"I_wheel: {I_wheel}")
print(f"TotalMassLever: {m_leverTotal}")
print(f"motor torque max: {max_motor_torque}")




#loop time
simulate_time = 10 # seconds
dt = 0.001 # seconds
theta = np.deg2rad(5) # initial angle (degrees)
theta_vel = 0 # initial angular velocity (rad/s)
time = 0
WheelV = 0 # initial wheel velocity (rad/s)
angles = []
timepoints = []
WheelVels = []
T_wheel = 0 # initial wheel torque
trueAngleDelta = angle_offset(x_dist, y_dist)
relativeV = 0
cumulative_error = 0 # For integral control of wheel speed

print(f"trueAngleDelta: {np.rad2deg(trueAngleDelta)}")
delayTime = 0.012 # s
delayCycles = int(delayTime/dt)
print(f"reading processing delayed by: {delayCycles} cycles ({delayCycles * dt} seconds)")

Arr_thetaMeasured = []
Arr_thetaVelMeasured = []
Arr_wheelVMeasured = []
Arr_cumulativeError = []

for i in range(0, delayCycles):
    Arr_thetaMeasured.append(np.deg2rad(theta))
    Arr_thetaVelMeasured.append(theta_vel)
    Arr_wheelVMeasured.append(WheelV)
    Arr_cumulativeError.append(cumulative_error)


while time < simulate_time and abs(np.rad2deg(theta)) < 45: # Stop if the cube falls over
    K = 36e-3
    T_grav = Grav_torque(m_leverTotal, theta, trueAngleDelta)
    T_dist = dist_torque(time, use = False)
    #T_wheel = 0
    
    wheel_angular_acceleration = wheel_AAc(T_wheel, I_wheel)
    WheelV += wheel_angular_acceleration * dt # Update wheel velocity based on angular acceleration
    WheelVels.append(60*WheelV/(2*np.pi)) #angular velocity in rpm


    body_angular_acceleration = body_AAc(T_grav, T_dist, T_wheel, Ip)
    theta_vel += body_angular_acceleration * dt # Update angular velocity based on angular acceleration
    theta += theta_vel * dt # Update angle based on angular velocity
    angles.append(np.rad2deg(theta))
    timepoints.append(time)
    relativeV = WheelV - theta_vel
    if abs(T_wheel) < max_motor_torque:
        cumulative_error += relativeV * dt # Update cumulative error for integral control - time multiplied in to allow for change in time step
    else:
        pass

    theta_measured = theta + random.uniform(np.deg2rad(-0.1), np.deg2rad(0.1)) # Add noise to the angle measurement
    theta_vel_measured = theta_vel + random.uniform(np.deg2rad(-1), np.deg2rad(1)) # Add noise to the angular velocity measurement
    WheelV_measured = WheelV + random.uniform(np.deg2rad(-1), np.deg2rad(1)) # Add noise to the wheel velocity measurement

    Arr_thetaMeasured.append(theta_measured)
    Arr_thetaVelMeasured.append(theta_vel_measured)
    Arr_wheelVMeasured.append(WheelV_measured)
    Arr_cumulativeError.append(cumulative_error)


    T_demand = pid_control(Arr_thetaMeasured[-delayCycles], Arr_thetaVelMeasured[-delayCycles], Arr_wheelVMeasured[-delayCycles], Arr_cumulativeError[-delayCycles], Kp = 3.3, Ki = 0.001, Kd = 0.23, Kp2 = 0.01, maxT = max_motor_torque, use = True)
    T_wheel, saturated = MD.delivered_torque(60*WheelV/(2*np.pi), T_demand/K, 6, K)
    time += dt # Update time
    if saturated:
        print("motor saturated")

fig, ax = plt.subplots(1, 2, figsize=(10, 5))
ax[0].plot(timepoints, angles); ax[0].set_title('Cube Angle vs Time'); ax[0].set_xlabel('Time (s)'); ax[0].set_ylabel('Angle (degrees)')
ax[1].plot(timepoints, WheelVels); ax[1].set_title('Wheel Velocity vs Time'); ax[1].set_xlabel('Time (s)'); ax[1].set_ylabel('Wheel Velocity (rpm)')

plt.show()

gui = False
if gui:
    root = tk.Tk()
    root.geometry("800x800")
    root.aspect(1, 1, 1, 1)
    canvas = tk.Canvas()
    graphCanvas = FigureCanvasTkAgg(fig, root)
    graphCanvas.draw()
    graphCanvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    canvas.create_rectangle(50, 50, 400, 400, fill="red", width = 2)
    canvas.pack(fill=tk.BOTH, expand=1)
    root.mainloop()






'''
Things to do for simulation:
- Add arduino torque change delay + sensor read delay
- Filtering of sensor data
- 3D case incl gyroscopic effects
- Visulisation panel
- Other control methods (LQR, MPC, etc.)
- Replace I, m etc with data from CAD
'''