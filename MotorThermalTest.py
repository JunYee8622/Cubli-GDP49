import ThermalChecks as TC
import matplotlib.pyplot as plt

current = 6
time = 0
Twind = TC.celcius_to_kelvin(25)
Thous = Twind
Tamb = Twind
DTM = 0
dt = 0.001

Arr_thermIlimit = []
Arr_Twind = []
Arr_times = []

while time < 60:
    thermILimit, Twind, Thous, DTM = TC.thermal_sim(current, Twind, Thous, Tamb, dt, DTM)
    Arr_times.append(time)
    Arr_Twind.append(Twind)
    Arr_thermIlimit.append(thermILimit)
    time += dt

fig, ax = plt.subplots(1, 2, figsize=(10, 5))
ax[0].plot(Arr_times, Arr_Twind); ax[0].set_title("Temperature change at max torque"); ax[0].set_xlabel("Time | s"); ax[0].set_ylabel("Winding Temperature | K")
ax[1].plot(Arr_times, Arr_thermIlimit); ax[1].set_title("Current Availability"); ax[1].set_xlabel("Time | s"); ax[1].set_ylabel("Thermal Current Limit | A")

plt.show()