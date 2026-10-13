import numpy as np
from matplotlib import pyplot as plt

NoLoadSpeed = 6250 #rpm
'''
StallTorque = 918e-3 #Nm
maxContinuousTorque = 110e-3 #Nm
TorqueConstant = 36e-3 #Nm/A

v_in = 24 #V
'''

def motor_characteristics(omegarpm = np.linspace(0, NoLoadSpeed, 100),NoLoadSpeed = 6250, StallTorque = 918e-3, maxContinuousTorque = 110e-3, plot=False, K = 36e-3):
    omega = omegarpm *(2*np.pi)/60
    wheelSpeed = abs(omega*60/(2*np.pi)) #rpm
    torque = (-StallTorque / NoLoadSpeed * wheelSpeed + StallTorque)
    torque = np.maximum(torque, 0)
    maxDriverCurrent = 6 #A
    maxContDriverCurrent = 2 #A
    maxContinuousTorqueDriver = maxContDriverCurrent * K
    maxDriverTorque = maxDriverCurrent * K

    if plot:
        plt.plot(wheelSpeed, torque, label='Torque Available')
        plt.axhline(y=maxContinuousTorque, color='r', linestyle='--', label=f'Max Continuous Torque (motor): {maxContinuousTorque*1000:.1f} mNm')
        plt.axhline(y=maxContinuousTorqueDriver, color='g', linestyle='--', label=f'Max Continuous Torque (Driver): {maxContinuousTorqueDriver*1000:.1f} mNm')
        plt.axhline(y=maxDriverTorque, color='b', linestyle='-', label=f'Max Torque (Driver): {maxDriverTorque*1000:.1f} mNm')
        plt.xlabel('Angular Velocity (rpm)')
        plt.ylabel('Torque (Nm)')
        plt.title('Motor Characteristics')
        plt.grid(True)
        plt.legend()
        plt.show()
    return(torque)

#plot_motor_characteristics(NoLoadSpeed, StallTorque, maxContinuousTorque)

def delivered_torque(motorSpeed, current, MaxCurrentDriver, TorqueConstant = 36e-3):
    wheelDirection = np.sign(motorSpeed)
    RequestedTorque = current * TorqueConstant
    MaxAvailableTorque = np.inf #Max torque derived from motor char is due to back EMF, and thus does not limit braking
    if np.sign(RequestedTorque) == wheelDirection:
        MaxAvailableTorque = motor_characteristics(omegarpm=motorSpeed)

    Limit = min(MaxAvailableTorque, MaxCurrentDriver * TorqueConstant)  #absolute thermal limit determined by the lower of the two
    if abs(RequestedTorque) >= Limit:
        return (np.sign(RequestedTorque) * Limit, True) #Returns max available torque and a saturation flag
    else:
        return (RequestedTorque, False) #Ensure that continuous limits are respected outside of this file.

#motor_characteristics(omegarpm=np.linspace(0, 1.2*NoLoadSpeed, 100), plot=True)