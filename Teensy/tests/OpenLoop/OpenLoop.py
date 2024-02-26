"""
wheel diameter = 72 mm
Gear ratio = 1:20
Ticks/motor_turn = 8192 
=> TICKS_TO_M = pi*72e-3/20/8192 = 1.38e-6

(1(t) (*) h(t)) = 8.551552e-01*(1-exp(-t/9.282536e-01))
Chosen response time in closed loop : 9.282536e-01
Kp, Ki : 1.169378e+00 1.259762e+00
"""

import numpy as np
import matplotlib.pyplot as plt

array = np.loadtxt("OpenLoop2.txt",dtype=np.float64,delimiter="\t")

time = (array[:,0]-array[3,0])*1e-3
speed = np.diff(array[:,1])/np.diff(time)*1.38e-6
print(np.column_stack([time[1:], speed]))
alpha = np.mean(speed[-70:])
beta = alpha*(time[4]-time[3])/(speed[4]-speed[3])
alpha *= 256/180
print(f"(1(t) (*) h(t)) = {format(alpha,'e')}*(1-exp(-t/{format(beta, 'e')}))")

tau = beta
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