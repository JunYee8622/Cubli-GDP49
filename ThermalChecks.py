import numpy as np
def heat_capacity(tau, Rth):
    return(tau/Rth)

def net_heat(current, Th, Tl, Rthermal, motor = (True, 0.942)):
    if motor[0]:
        Rterminal = motor[1]
        Qin = current**2 * Rterminal
    else:
        Qin = motor[1]
    Qout = (Th - Tl)/Rthermal
    Qnet = Qin - Qout
    return(Qout, Qnet)

def celcius_to_kelvin(temp):
    return (temp + 273.15)


def update_temp(Qnet, c, T0, dt):
    Tnew = T0 + (Qnet / c)*dt
    return(Tnew)

def driver_temp_monitor(current, DTM, dt, tau =34):
    newDTM = DTM * np.exp(-dt/tau) + current**2 * (1- np.exp(-dt/tau)) 
    return(newDTM)


tau_winding = 26.7 #s
Tres_windingHousing = 7.05 #K/W
tau_motor = 13.4 #s
Tres_housingAmbient = 0.268 # K/W
Tmax_winding = 125 # celcius

IMotornom = 3
IMotormax = 6

C_winding = heat_capacity(tau_winding, Tres_windingHousing)
C_housing = heat_capacity(tau_motor, Tres_housingAmbient)

IDrivernom = 2
IDriverMax = 6
t_IDriverMax = 4 #s
def thermal_sim(current, Twinding, Thousing, Tamb, dt, DTM):
    
    '''Motor to housing heat transfer'''
    QoutWinding, QnetWinding = net_heat(current, Twinding, Thousing, Tres_windingHousing)
    TnewWinding = update_temp(QnetWinding, C_winding, Twinding, dt)

    '''housing to ambient heat transfer'''
    QinHousing = QoutWinding
    QnetHousing = net_heat(current, Thousing, Tamb, Tres_housingAmbient, (False, QinHousing))[1]
    TnewHousing = update_temp(QnetHousing, C_housing, Thousing, dt)


    updatedDTM = driver_temp_monitor(current, DTM, dt)
    if updatedDTM <= 0.8*IDrivernom**2: #held at 2A (I**2 relationship)
        IdriverLimit = IDriverMax
    elif updatedDTM <= IDrivernom**2:
        IdriverLimit = (IDrivernom - IDriverMax)/(IDrivernom**2 - 0.8*IDrivernom**2) *(updatedDTM - IDrivernom**2) + IDrivernom
    else:
        IdriverLimit = IDrivernom

    if TnewWinding <= celcius_to_kelvin(0.8 * Tmax_winding):
        return(min(IMotormax, IdriverLimit), TnewWinding, TnewHousing, updatedDTM)
    else:
        Imax = ((IMotornom - IMotormax)/(0.95* Tmax_winding - 0.8 * Tmax_winding)) * (TnewWinding - (celcius_to_kelvin(Tmax_winding*0.8))) + IMotormax 
        if Imax < 0:
            Imax = 0
        return(min(Imax, IdriverLimit), TnewWinding, TnewHousing, updatedDTM)