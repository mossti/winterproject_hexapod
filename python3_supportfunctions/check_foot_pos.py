#!/usr/bin/env python

#from adafruit_servokit import ServoKit
import time
import numpy as np
import modern_robotics as mrcl

# Define directional vector ('forward/backward' vector for linear motion with angle and magnitude)
direcvec = float(input('Angle off-norm for linear motion: ')) ## this should change along with input direction (e.g. from joystick)
direcmagnitude = 1 ## 1 is normalized speed, with 0 stopped and 2 maximum speed
direcvec = np.array([direcvec, direcmagnitude])

# Angle variables (initial values for normal 'forward' x-axis and 90/90 hip/knee angles)
## base to HIP (mechanically fixed but will change w.r.t. directional vector)
theta0 = (30 - direcvec[0])*((np.pi)/180)

## hip to KNEE (each leg 60 degrees from one another)
phi0 = 90*((np.pi)/180)
## knee to LEG ('splayed')
psi0 = 90*((np.pi)/180)

## Angle lists (per leg)
print("\n-------------------------\nAngle list (single leg):\n-------------------------\n")
legAL_0 = np.array([theta0, phi0, psi0])
print(legAL_0)

## Body screw axes
B_basetoHIP_0 = np.array([0, 0, 1, 0.056803+0.04923984+0.0644906, 0, -0.009611-0.0357124])
B_hiptoKNEE_0 = np.array([0, 0, 1, 0.04923984+0.0644906, 0, -0.0357124])
B_kneetoEE_0 = np.array([0, 1, 0, 0.0644906, 0, -0.0357124])
B_list_0 = np.array([B_basetoHIP_0, B_hiptoKNEE_0, B_kneetoEE_0])

## [[0]] Transformation matrix from radial center to EE0 at [30/90/90]
T_basetoHIP_0 = np.array([[np.cos(theta0), np.sin(theta0), 0, 0.057803],
                         [-(np.sin(theta0)), np.cos(theta0), 0, 0],
                         [0, 0, 1, -0.009611],
                         [0, 0, 0, 1]])
T_hiptoKNEE_0 = np.array([[np.cos(phi0), np.sin(phi0), 0, 0.04923984],
                         [-(np.sin(phi0)), np.cos(phi0), 0, 0],
                         [0, 0, 1, 0], ## need to CHECK this z-displacement (!!!)
                         [0, 0, 0, 1]])
T_kneetoEE_0 = np.array([[np.cos(psi0), 0, np.sin(psi0), 0.0644906],
                         [0, 1, 0, 0],
                         [-(np.sin(psi0)), 0, np.cos(psi0), -0.0357124],
                         [0, 0, 0, 1]])

print("\n--------------\nT_basetoEE_0:\n--------------\n")
T_basetoKNEE_0 = np.dot(T_basetoHIP_0,T_hiptoKNEE_0)
T_basetoEE_0 = np.dot(T_basetoKNEE_0,T_kneetoEE_0)
print(T_basetoEE_0)

print("\n----\nM_0:\n----\n")
M_0 = mrcl.MatrixLog6(T_basetoEE_0) ## se(3) representation of exponential coordinates
print(M_0)
