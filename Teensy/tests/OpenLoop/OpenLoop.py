"""
wheel diameter = 72 mm
Gear ratio = 1:20
Ticks/motor_turn = 8192 
=> TICKS_TO_M = pi*72e-3/20/8192 = 1.38e-6

(1(t) (*) h(t)) = 1.282733e+00*(1-exp(-t/9.282536e-01))
Chosen response time in closed loop : 9.282536e-01
Kp, Ki : 7.795856e-01 8.398411e-01
"""

import numpy as np
import matplotlib.pyplot as plt

array = np.loadtxt("OpenLoop2.txt",dtype=np.float64,delimiter="\t")

time = (array[:,0]-array[3,0])*1e-3
speed = np.diff(array[:,1])/np.diff(time)*1.38e-6
print(np.column_stack([time[1:], speed]))
alpha = np.mean(speed[-70:])*256/120
beta = alpha*120/256*(time[4]-time[3])/(speed[4]-speed[3])
print(f"(1(t) (*) h(t)) = {format(alpha,'e')}*(1-exp(-t/{format(beta, 'e')}))")

tau = beta/2
print(f"Chosen response time in closed loop : {format(tau,'e')}")

Kp = beta/alpha/tau
Ki = 1/alpha/tau
print("Kp, Ki :", format(Kp, "e"), format(Ki, "e"))

plt.figure()
plt.plot(time[1:], speed)
plt.plot([beta,time[-1]], [speed[-1], speed[-1]], "-r")
plt.plot([0,beta], [0, speed[-1]], "-r")
plt.plot([beta,beta], [speed[-1],0], "--r")
plt.title("Open Loop Step Response")
plt.xlabel("Time [s]")
plt.ylabel("Speed [m/s]")
plt.grid(True)
plt.show()