import BodyDyn as BD
import scipy.optimize as optimize
import numpy as np

def angle_error_abs(gains):
    WheelVweight = 1e-9
    peakreductionweight = 0.2
    kp1, ki, kd, kp2 = gains
    angledelta = BD.angle_offset(BD.x_dist, BD.y_dist)
    simtime = 10
    dt = 0.001
    Arr_timepoints, Arr_angles, Arr_wheelAngle, WheelVels = BD.simulate(True, simtime, dt, 0, 0.015, (kp1, ki, kd, kp2))
    if Arr_timepoints[-1] < (0.95*simtime):
        return(1e12 * (simtime - Arr_timepoints[-1]))
    else:
        summedError = 0
        for i in range(0, len(Arr_angles)):
            #(abs(Arr_angles[i] - np.rad2deg(angledelta)) *peakreductionweight* dt  + (WheelVels[i])**2 * WheelVweight)
            summedError += (abs(Arr_angles[i] - np.rad2deg(angledelta)) ) *dt * Arr_timepoints[i]
        return(summedError)

initialguess = [3.3, 0.001, 0.22, 0.001]

result = optimize.minimize(angle_error_abs, initialguess, bounds =[(0, 100),(0, 100),(0, 100),(0, 100)], method = 'Nelder-Mead')

print(result.x)
print(result.nfev)
print(result.message)
print(result.fun)



#results: [3.90715701e+00 1.42876708e-07 5.34541682e-01 3.74546800e-03] - works, wheel speed does not tend to 0
#[5.65940382e+00 0.00000000e+00 5.52525324e-01 1.13033501e-03]
#[8.61817371e+00 2.47127648e-05 1.25690132e+00 1.35912411e-02]