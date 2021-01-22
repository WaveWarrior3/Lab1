# -*- coding: utf-8 -*-
"""
Servomotor Nonlinear PWM Response

@author: Ethan Uetrecht
"""

import numpy as np
import matplotlib.pyplot as plt

dbWidth = 90
pwMin = 700
pwMax = 2300

tanScale = [0.5, 1, 2, 4, 8]
#tanScale = [1]

#pwmWithDB = np.arange(pwMin,pwMax,dbWidth)
#RPMwithDB = 100*np.tanh((pwmWithDB-1500)/800*np.pi/scale)
#plt.plot(pwmWithDB,RPMwithDB)

for s in np.arange(len(tanScale)):
    pwm = np.arange(-1,1,0.01)
    RPM = np.tanh(pwm*np.pi/(tanScale[s]))
    RPM = RPM * 100/RPM[-1]
    plt.plot(pwm,RPM)
plt.xlabel('PWM Duty Cycle')
plt.ylabel('Servo Speed (RPM)')
plt.title('Servo Nonlinear PWM Response')