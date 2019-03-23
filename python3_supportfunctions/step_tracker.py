#!/usr/bin/env python

#from adafruit_servokit import ServoKit
import time
import numpy as np
import modern_robotics as mrcl

'''
kit = ServoKit(channels=16)

def hip_move(tripod,user_angle):
	ii=tripod
	while ii<=5:
    if ii==0
		kit.servo[ii].angle = int(user_angle)
		ii+=2

def knee_move(tripod,user_angle):
	jj=tripod+6
	while jj<=11:
		kit.servo[jj].angle = int(user_angle)
		jj+=2
'''

def steptracker():
	# Define directional vector ('forward/backward' vector for linear motion with angle and magnitude)
	direcvec = float(input('Angle off-norm for linear motion: ')) ## this should change along with input direction (e.g. from joystick)
	direcmagnitude = 1 ## 1 is normalized speed, with 0 stopped and 2 maximum speed
	direcvec = np.array([direcvec, direcmagnitude])

	# Angle variables (initial values for normal 'forward' x-axis and 90/90 hip/knee angles)
	## base to HIP (mechanically fixed but will change w.r.t. directional vector)
	theta0 = (30 - direcvec[0])*((np.pi)/180)

	# index for phi0 and psi0
	i = 0

	while True:

		if i == 0:
			## hip to KNEE (each leg 60 degrees from one another)
			phi0 = 0*((np.pi)/180)
			## knee to LEG ('splayed')
			psi0 = 0*((np.pi)/180)
		if i != 0:
			## hip to KNEE (each leg 60 degrees from one another)
			phi0 = test_thetalist_0[1]*((np.pi)/180)
			## knee to LEG ('splayed')
			psi0 = test_thetalist_0[2]*((np.pi)/180)

		## Body screw axes
		B_basetoHIP_0 = np.array([0, 0, 1, 0, 0, 0])
		B_hiptoKNEE_0 = np.array([0, 0, 1, 0.057803, 0, 0])
		B_kneetoEE_0 = np.array([0, 1, 0, 0, 0, 0.04923984+0.057803])

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

		T_basetoKNEE_0 = np.dot(T_basetoHIP_0,T_hiptoKNEE_0)
		T_basetoEE_0 = np.dot(T_basetoKNEE_0,T_kneetoEE_0)		

		hipjoint = float(input('\n>> Please enter hip joint angle for TTM: '))*((np.pi)/180)
		print(hipjoint)
		kneejoint = float(input('>> Please enter knee joint angle for TTM: '))*((np.pi)/180)
		print(kneejoint)
		test_thetalist_0 = np.array([theta0, hipjoint, kneejoint])

		print("\n--------------------------------\nTest Transformation Matrix:\n--------------------------------\n")
		test_transform_0 = mrcl.FKinBody(T_basetoEE_0, B_list_0.T, test_thetalist_0)
		if i == 0:
			print('\nPrevious transform: \n')
			print(T_basetoEE_0)
			test_transform_0_temp = test_transform_0
		else:
			print('\nPrevious transform: \n')
			print(test_transform_0_temp)
			test_transform_0_temp = test_transform_0
		print("\nCurrent transform: \n")
		print(test_transform_0)

		i+=1

try:
	steptracker()
except KeyboardInterrupt:
	print('Exiting.')
	exit()
