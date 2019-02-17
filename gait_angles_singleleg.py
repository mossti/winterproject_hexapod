#!/usr/bin/env python

#from adafruit_servokit import ServoKit
import time
import numpy as np
import modern_robotics as mrcl

#kit = ServoKit(channels=16)

# Masses (kg)
m_base = 0.09563713
m_hip = 0.01143887
m_leg = 0.01355592
m_servo = 0.00850905

# Surface areas (m^2)
A_base = 0.08356029
A_hip = 0.00689779
A_leg = 0.00817656
A_servo = 0.00298872

# Volumes (m^3)
Vol_base = 0.00007651
Vol_hip = 0.00000915
Vol_leg = 0.00001084
Vol_servo = 0.00000715

# Inertial matrices (kg/m^2)
J_base = np.array([[1.74724793e-4, -2.17826739e-6, 7.50299954e-7],
                   [-2.17826739e-6, 1.53740737e-4, -2.18772147e-7],
                   [7.50299954e-7, -2.18772147e-7, 2.71515931e-4]])
J_hip = np.array([[3.37960510e-6, -5.28556628e-9, 5.28556628e-9], # for full hip (2 joined)
                  [-5.28556628e-9, 3.85155891e-6, -3.49717596e-9],
                  [5.28556628e-9, -3.49717596e-9, 3.85155891e-6]])
J_leg = np.array([[5.26673919e-6, -3.87781653e-8, -1.44676781e-6],
                  [-3.87781653e-8, 5.76203671e-6, -2.02146715e-7],
                  [-1.44676781e-6, -2.02146715e-7, 2.10331450e-6]])
J_servo = np.array([[5.99159296e-7, -4.80736752e-8, 7.35154348e-13],
                    [-4.80736752e-8, 4.95518845e-7, 1.43035134e-11],
                    [7.35154348e-13, 1.43035134e-11, 8.97966511e-7]])

# Define directional vector ('forward/backward' vector for linear motion with angle and magnitude)
direcvec = float(input('Angle off-norm for linear motion: ')) ## this should change along with input direction (e.g. from joystick)
direcmagnitude = 1 ## 1 is normalized speed, with 0 stopped and 2 maximum speed
direcvec = np.array([direcvec, direcmagnitude])

# Angle variables (initial values for normal 'forward' x-axis and 90/90 hip/knee angles)
## base to HIP (mechanically fixed but will change w.r.t. directional vector)
theta0 = (30 - direcvec[0])*((np.pi)/180)

## hip to KNEE (each leg 60 degrees from one another)
phi0 = 0*((np.pi)/180)
## knee to LEG ('splayed')
psi0 = 0*((np.pi)/180)

## Angle lists (per leg)
print("\n-------------------------\nAngle list (single leg):\n-------------------------\n")
legAL_0 = np.array([theta0, phi0, psi0])
print(legAL_0)

## Body screw axes
B_basetoHIP_0 = np.array([0, 0, 1, 0, 0, 0])
B_hiptoKNEE_0 = np.array([0, 0, 1, 0.057803, 0, 0])
B_kneetoEE_0 = np.array([0, 1, 0, 0, 0, 0.04923984+0.057803])
#S_basetoHIP_0 = np.array([0, 0, 1, -0.056803*np.sin(theta0), 0.056803*np.cos(theta0), 0])
#S_hiptoKNEE_0 = np.array([0, 0, 1, -0.04923984*np.sin(phi0), 0.04923984*np.cos(phi0), 0])
#S_kneetoEE_0 = np.array([0, 1, 0, -0.0644906*np.sin(psi0), 0, 0.0644906*np.cos(psi0)])
#B_basetoHIP_0 = np.array([0, 0, 1, 0.056803+0.04923984+0.0644906, 0, -0.009611-0.0357124])
#B_hiptoKNEE_0 = np.array([0, 0, 1, 0.04923984+0.0644906, 0, -0.0357124])
#B_kneetoEE_0 = np.array([0, 1, 0, 0.0644906, 0, -0.0357124])
B_list_0 = np.array([B_basetoHIP_0, B_hiptoKNEE_0, B_kneetoEE_0])

## [[0]] Transformation matrix from radial center to EE0 at [30/90/90]
T_basetoHIP_0 = np.array([[np.cos(theta0), np.sin(theta0), 0, 0.0578104],
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

T_restingEE_0 = np.array([[0],
                          [0],
                          [0],
                          [0]])

print("\n------------------------\nMatrix Log [se(3) representation]:\n------------------------\n")
M_0 = mrcl.MatrixLog6(T_basetoEE_0) ## se(3) representation of exponential coordinates
print(M_0)

hipjoint = float(input('\n>> Please enter hip joint angle for TTM: '))*((np.pi)/180)
print(hipjoint)
kneejoint = float(input('>> Please enter knee joint angle for TTM: '))*((np.pi)/180)
print(kneejoint)
test_thetalist_0 = np.array([theta0, hipjoint, kneejoint])
print("\n------------\nS_list_0: \n------------\n")
print(B_list_0)
print("\n-----------------\ntest_thetalist_0: \n-----------------\n")
print(test_thetalist_0)

print("\n--------------------------------\nTest Transformation Matrix:\n--------------------------------\n")
test_transform_0 = mrcl.FKinBody(T_basetoEE_0, B_list_0.T, test_thetalist_0)
print(test_transform_0)
#[thetalist,success] = mrcl.IKinBody()

print("\n--------------------------------\nJacobian Body:\n--------------------------------\n")
J_body = mrcl.JacobianBody(B_list_0.T, test_thetalist_0)
print(J_body)

print("\n--------------------------------------\nEE x/y/z distance displaced from rest: \n--------------------------------------")
L0x = 0.0578104
L0z = -0.009611 #positive in space
L0 = np.sqrt(0.0578104**2+(-0.009611)**2)
L1 = 0.04923984
L2 = np.sqrt(0.0644906**2+(-0.0357124)**2)
currposz = L0z - L2*np.sin(kneejoint-50) #subtract from psi0 to accomodate curvature of leg
currposy = L0x*np.sin(theta0) + (L1+L2*np.cos(kneejoint-50))*np.sin(hipjoint)
currposx = L2*np.cos(kneejoint-50) + L1*(np.cos(theta0)) + L0x*np.cos(theta0)
print("\ndist. x: ")
print(currposx)
print("\ndist. y: ")
print(currposy)
print("\ndist. z: ")
print(currposz)

#traj = JointTrajectory(thetastart,thetaend,Tf,N,method)
