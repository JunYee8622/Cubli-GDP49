#maximum motor accel test
import MotorDynamics as MD
import numpy as np
import matplotlib.pyplot as plt

def wheel_AAc(T, I): #angular acceleration of wheel
    return(T/I)

I_wheel = 75889.50e-9 # from plastic version CAD m1

K = 36e-3
Icontlimit = 2*K


MaxDriverCurrent = 6 #A

#Targets = [0.9e-3, 0.2e-3, 0.1e-3]
Targets = [np.inf]
colcount = 0
fig, ax = plt.subplots(1, 3, figsize=(10, 5))
Treceived = 0
for T_target in Targets:
    Vwheel = 0
    Arr_Vwheel = []
    Arr_Saturated = []
    Arr_time = []
    Arr_Treceived = []
    linecol = ["r", "g", "b", "m"]
    Treceived = 0
    saturated = False
    t = 0
    dt = 1e-2
    t_high = 0
    DriverTorqueLimit = False
    while t < 1:
        WheelAcc = wheel_AAc(Treceived, I_wheel)
        Vwheel += WheelAcc * dt
        Arr_Vwheel.append(60*Vwheel/(2*np.pi))
        Arr_time.append(t)
        Arr_Treceived.append(Treceived)
        # if DriverTorqueLimit:
        #     t_high -= dt
        #     if t_high <= 0
        # if Treceived > Icontlimit: #Driver continuous limits
        #     t_high += dt
        # else:
        #     t_high -= dt
        # if t_high < 0:
        #     t_high = 0
        # if t_high >= 3.5:
        #     DriverTorqueLimit = True

        if saturated:
            Arr_Saturated.append(1)
        else:
            Arr_Saturated.append(0)
        (Treceived, saturated) = MD.delivered_torque((Vwheel*60)/(2*np.pi), T_target/K, MaxDriverCurrent, K)
        if DriverTorqueLimit == True and Treceived >= 2*K:
            Treceived = 2*K
        t += dt

    ax[0].plot(Arr_time, Arr_Vwheel, color = linecol[colcount]); ax[0].set_title("Wheel speed under max torque"); ax[0].set_xlabel("time"); ax[0].set_ylabel("Wheel Speed (rpm)")
    ax[1].plot(Arr_time, Arr_Saturated, color = linecol[colcount]); ax[1].set_title("Saturated flag"); ax[1].set_xlabel("time"); ax[1].set_ylabel("Saturated")
    ax[2].plot(Arr_time, Arr_Treceived, color = linecol[colcount]); ax[2].set_title("Torque Received"); ax[2].set_xlabel("time"); ax[2].set_ylabel("Torque")
      
    colcount +=1

plt.show()
