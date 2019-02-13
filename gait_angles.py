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
