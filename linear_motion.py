#!/usr/bin/env python

#from adafruit_servokit import ServoKit
import time
import numpy as np
import modern_robotics as mrcl

#kit = ServoKit(channels=16)

def tripod_hip_move(tripod,user_angle):
	ii=tripod
	while ii<=5:
		kit.servo[ii].angle = int(user_angle)
		ii+=2

def tripod_knee_move(tripod,user_angle):
	jj=tripod+6
	while jj<=11:
		kit.servo[jj].angle = int(user_angle)
		jj+=2

def reset_purestance():
  tripod_hip_move(0,90)
  tripod_hip_move(1,90)
  tripod_knee_move(0,30)
  tripod_knee_move(1,30)

def forward_kinematics():

def inverse_kinematics():

def control_input():

def add_direcvec():

def workspace_trim():

def calculate_gait():

def drive_servos():






try:
	linear_motion()
except KeyboardInterrupt:
  reset_purestance()
	print('\nExiting.')
	exit()
