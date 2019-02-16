from adafruit_servokit import ServoKit
import time
import numpy as np
import modern_robotics as mrcl

kit = ServoKit(channels=16)

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
direcvec = 0 ## this should change along with input direction (e.g. from joystick)
direcmagnitude = 1 ## 1 is normalized speed, with 0 stopped and 2 maximum speed
direcvec = np.array([direcvec, direcmagnitude])

# Angle variables (initial values for normal 'forward' x-axis and 90/90 hip/knee angles)
## base to HIP (mechanically fixed but will change w.r.t. directional vector)
theta0 = (30 - direcvec[0])*(np.pi/180)
theta1 = (90 - direcvec[0])*(np.pi/180)
theta2 = (150 - direcvec[0])*(np.pi/180)
theta3 = (210 - direcvec[0])*(np.pi/180)
theta4 = (270 - direcvec[0])*(np.pi/180)
theta5 = (330 - direcvec[0])*(np.pi/180)
## hip to KNEE (each leg 60 degrees from one another)
phi0 = (90)*(np.pi/180)
phi1 = (90)*(np.pi/180)
phi2 = (90)*(np.pi/180)
phi3 = (90)*(np.pi/180)
phi4 = (90)*(np.pi/180)
phi5 = (90)*(np.pi/180)
## knee to LEG ('splayed')
psi0 = (90)*(np.pi/180)
psi1 = (90)*(np.pi/180)
psi2 = (90)*(np.pi/180)
psi3 = (90)*(np.pi/180)
psi4 = (90)*(np.pi/180)
psi5 = (90)*(np.pi/180)

# Angle lists
## Angle lists (radial)
rAL_basetoHIP = np.array([theta0, theta1, theta2, theta3, theta4, theta5]) #these will change
rAL_hiptoKNEE = np.array([phi0, phi1, phi2, phi3, phi4, phi5])
rAL_kneetoLEG = np.array([psi0, psi1, psi2, psi3, psi4, psi5])
## Angle lists (per leg)
legAL_0 = np.array([theta0, phi0, psi0])
legAL_1 = np.array([theta1, phi1, psi1])
legAL_2 = np.array([theta2, phi2, psi2])
legAL_3 = np.array([theta3, phi3, psi3])
legAL_4 = np.array([theta4, phi4, psi4])
legAL_5 = np.array([theta5, phi5, psi5])

# Transformation matrices
## [[0]] Transformation matrix from radial center to EE0
T_basetoHIP_0 = np.array([][np.cos(theta0), np.sin(theta0), 0, 0.057803],
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
T_basetoEE_0 = np.dot(np.dot(T_basetoHIP_0,T_hiptoKNEE_0),T_kneetoEE_0)
M_0 = mrcl.MatrixLog6(T_basetoEE_0) ## se(3) representation of exponential coordinates
## [[1]] Transformation matrix from radial center to EE1
T_basetoHIP_1 = np.array([[np.cos(theta1), np.sin(theta1), 0, 0.057803],
                         [-(np.sin(theta1)), np.cos(theta1), 0, 0],
                         [0, 0, 1, -0.009611],
                         [0, 0, 0, 1]])
T_hiptoKNEE_1 = np.array([[np.cos(phi1), np.sin(phi1), 0, 0.04923984],
                         [-(np.sin(phi1)), np.cos(phi1), 0, 0],
                         [0, 0, 1, 0], ## need to CHECK this z-displacement (!!!)
                         [0, 0, 0, 1]])
T_kneetoEE_1 = np.array([[np.cos(psi1), 0, np.sin(psi1), 0.0644906],
                         [0, 1, 0, 0],
                         [-(np.sin(psi1)), 0, np.cos(psi1), -0.0357124],
                         [0, 0, 0, 1]])
## [[2]] Transformation matrix from radial center to EE2
T_basetoHIP_2 = np.array([[np.cos(theta2), np.sin(theta2), 0, 0.057803],
                         [-(np.sin(theta2)), np.cos(theta2), 0, 0],
                         [0, 0, 1, -0.009611],
                         [0, 0, 0, 1]])
T_hiptoKNEE_2 = np.array([[np.cos(phi2), np.sin(phi2), 0, 0.04923984],
                         [-(np.sin(phi2)), np.cos(phi2), 0, 0],
                         [0, 0, 1, 0], ## need to CHECK this z-displacement (!!!)
                         [0, 0, 0, 1]])
T_kneetoEE_2 = np.array([[np.cos(psi2), 0, np.sin(psi2), 0.0644906],
                         [0, 1, 0, 0],
                         [-(np.sin(psi2)), 0, np.cos(psi2), -0.0357124],
                         [0, 0, 0, 1]])
## [[3]] Transformation matrix from radial center to EE3
T_basetoHIP_3 = np.array([[np.cos(theta3), np.sin(theta3), 0, 0.057803],
                         [-(np.sin(theta3)), np.cos(theta3), 0, 0],
                         [0, 0, 1, -0.009611],
                         [0, 0, 0, 1]])
T_hiptoKNEE_3 = np.array([[np.cos(phi3), np.sin(phi3), 0, 0.04923984],
                         [-(np.sin(phi3)), np.cos(phi3), 0, 0],
                         [0, 0, 1, 0], ## need to CHECK this z-displacement (!!!)
                         [0, 0, 0, 1]])
T_kneetoEE_3 = np.array([[np.cos(psi3)), 0, np.sin(psi3), 0.0644906],
                         [0, 1, 0, 0],
                         [-(np.sin(psi3), 0, np.cos(psi3), -0.0357124],
                         [0, 0, 0, 1]])
## [[4]] Transformation matrix from radial center to EE4
T_basetoHIP_4 = np.array([[np.cos(theta4), np.sin(theta4), 0, 0.057803],
                         [-(np.sin(theta4)), np.cos(theta4), 0, 0],
                         [0, 0, 1, -0.009611],
                         [0, 0, 0, 1]])
T_hiptoKNEE_4 = np.array([[np.cos(phi4), np.sin(phi4), 0, 0.04923984],
                         [-(np.sin(phi4)), np.cos(phi4), 0, 0],
                         [0, 0, 1, 0], ## need to CHECK this z-displacement (!!!)
                         [0, 0, 0, 1]])
T_kneetoEE_4 = np.array([[np.cos(psi4), 0, np.sin(psi4), 0.0644906],
                         [0, 1, 0, 0],
                         [-(np.sin(psi4)), 0, np.cos(psi4), -0.0357124],
                         [0, 0, 0, 1]])
## [[5]] Transformation matrix from radial center to EE5
T_basetoHIP_5 = np.array([[np.cos(theta5), np.sin(theta5), 0, 0.057803],
                         [-(np.sin(theta5)), np.cos(theta5), 0, 0],
                         [0, 0, 1, -0.009611],
                         [0, 0, 0, 1]])
T_hiptoKNEE_5 = np.array([[np.cos(phi5), np.sin(phi5), 0, 0.04923984],
                         [-(np.sin(phi5)), np.cos(phi5), 0, 0],
                         [0, 0, 1, 0], ## need to CHECK this z-displacement (!!!)
                         [0, 0, 0, 1]])
T_kneetoEE_5 = np.array([[np.cos(psi5), 0, np.sin(psi5), 0.0644906],
                         [0, 1, 0, 0],
                         [-(np.sin(psi5)), 0, np.cos(psi5), -0.0357124],
                         [0, 0, 0, 1]])

def establish_gait(direcvec):
