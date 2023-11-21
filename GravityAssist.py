import matplotlib.pyplot as plt
import numpy as np
import math
from decimal import Decimal
import time

MoonMass = 7.3*(10**22)
EarthMass = 5.97*(10**24) 
RocketMass = 10**4

velocityX = np.array([2e4])
velocityY = np.array([2e4])

positionX = np.array([4525.5])
positionY = np.array([4525.5])

deltaT = 20000
steps = 20000

MoonR = 1.7e6

aimDist = float(input("Impact parameter: "))

MoonXOff = aimDist * np.cos(0.785)
MonYOff = aimDist * np.sin(0.785)

MoonX = 2.7e8 + MoonXOff
MoonY = 2.7e8 - MonYOff

G =  6.674*(10**(-11))

distance = np.array([3.8e+8])
acceleration = np.array([0])

Time = np.array([0])

velocity = np.array([math.sqrt(velocityX[0]**2 + velocityY[0]**2)])

distanceFlew = 0
x = 4e8

i = 0

MoonXArr = np.array([MoonX])
MoonYArr = np.array([MoonY])

moonSpeed = 0

while np.absolute(distanceFlew) < x:
    deltaT = 200000000/(velocity[i]**2)

    Xm = MoonX - positionX[i]
    Ym = MoonY - positionY[i]

    distS = np.sqrt(Xm**2 + Ym**2)

    distance = np.append(distance, distS)
    distance_Earth = np.sqrt(positionX[i]**2 + positionY[i]**2)

    Force = (((G*RocketMass*MoonMass)/(distS**2))) # - ((G*RocketMass*EarthMass)/(distance_Earth**2))

    cosA = Xm/distS
    sinA = Ym/distS

    accelerationX = (cosA*Force)/RocketMass
    accelerationY = (sinA*Force)/RocketMass

    DVelocityX = accelerationX*deltaT
    DVelocityY = accelerationY*deltaT

    velocityX = np.append(velocityX, velocityX[i] + DVelocityX)
    velocityY = np.append(velocityY, velocityY[i] + DVelocityY)

    positionX = np.append(positionX, positionX[i] + velocityX[i+1]*deltaT)
    positionY = np.append(positionY, positionY[i] + velocityY[i+1]*deltaT)

    accelerationI = G*MoonMass/distS
    
    acceleration = np.append(acceleration, accelerationI)

    Time = np.append(Time, Time[i] + deltaT)

    velocity = np.append(velocity, math.sqrt(velocityX[i]**2 + velocityY[i]**2))

    distanceFlew += velocity[i] * deltaT

    percnt = int(distanceFlew/x*100)

    if Time[i] + deltaT > 13000:
        moonSpeed = 20

    MoonX = MoonXArr[i] + moonSpeed*deltaT
    MoonXArr = np.append(MoonXArr, MoonX)

    MoonY = MoonYArr[i] #+ 0.5*deltaT
    MoonYArr = np.append(MoonYArr, MoonY)

    print(percnt)

    i += 1

print(Time[-1])

plt.plot(positionX, positionY, label='Spacecraft', marker = '.',  color='b')
plt.plot(MoonXArr, MoonYArr, label='Moon', marker = '.',  color='r')

plt.xlabel('X-position')
plt.ylabel('Y-position')
plt.legend()

plt.show()

fig, axs = plt.subplots(3, 1, figsize=(6, 8), sharex=True)
fig.subplots_adjust(hspace=0)

axs[0].plot(Time, velocity, label="Line 1")
axs[1].plot(Time, distance, label="Line 2")
axs[2].plot(Time, acceleration, label="Line 3")

axs[0].set_ylabel('velocity, m/s')
axs[1].set_ylabel('distance, m')
axs[2].set_ylabel('acceleration, m/s^2')
axs[2].set_xlabel('time, s')

plt.tight_layout()

plt.show()
