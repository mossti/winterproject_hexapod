#!/usr/bin/env python

#from adafruit_servokit import ServoKit
import time
import numpy as np
import modern_robotics as mrcl

#kit = ServoKit(channels=16)

## Body screw axes
B_basetoHIP = np.array([0, 0, 1, 0, 0, 0])
B_hiptoKNEE = np.array([0, 0, 1, 0.057803, 0, 0])
B_kneetoEE = np.array([0, 1, 0, 0, 0, 0.04923984+0.057803])
B_list = np.array([B_basetoHIP, B_hiptoKNEE, B_kneetoEE])


## hip to KNEE (each leg 60 degrees from one another)
phinot = 0*((np.pi)/180)
## knee to LEG (30 degrees on servo ~ CHECK!!)
psinot = -60*((np.pi)/180)
anglelist = np.array([phinot,psinot])
# Angle list
anglelistFull = np.array([[phinot,psinot],[phinot,psinot],[phinot,psinot],[phinot,psinot],[phinot,psinot],[phinot,psinot]])
#np.array([[phi0,psi0],[phi1,psi1],[phi2,psi2],[phi3,psi3],[phi4,psi4],[phi5,psi5]])
riseheight = 0*((np.pi)/180)

## base to HIP (mechanically fixed)
theta0 = (30)*(np.pi/180)
theta1 = (90)*(np.pi/180)
theta2 = (150)*(np.pi/180)
theta3 = (210)*(np.pi/180)
theta4 = (270)*(np.pi/180)
theta5 = (330)*(np.pi/180)
thetalist = np.array([theta0,theta1,theta2,theta3,theta4,theta5])

def homeconfig(theta, phi,psi):
	## Transformation matrix from radial center to EEx
	T_basetoHIP = np.array([[np.cos(theta), np.sin(theta), 0, 0.0578104],
	                         [-(np.sin(theta)), np.cos(theta), 0, 0],
	                         [0, 0, 1, -0.009611],
	                         [0, 0, 0, 1]])
	T_hiptoKNEE = np.array([[np.cos(phi), np.sin(phi), 0, 0.04923984],
	                         [-(np.sin(phi)), np.cos(phi), 0, 0],
	                         [0, 0, 1, 0], ## need to CHECK this z-displacement (!!!)
	                         [0, 0, 0, 1]])
	T_kneetoEE = np.array([[np.cos(psi), 0, np.sin(psi), 0.0644906],
	                         [0, 1, 0, 0],
	                         [-(np.sin(psi)), 0, np.cos(psi), -0.0357124],
	                         [0, 0, 0, 1]])
	T_basetoKNEE = np.dot(T_basetoHIP,T_hiptoKNEE)
	T_basetoEE = np.dot(T_basetoKNEE,T_kneetoEE)
	return T_basetoEE



'''
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
'''

# credit to mcrl
def TransToRp(T):
	T = np.array(T)
	return T[0: 3, 0: 3], T[0: 3, 3]

# credit to mcrl
def NearZero(z):
	return abs(z) < 1e-6

# credit to mcrl
def TransInv(T):
	R,p = TransToRp(T)
	Rt = np.array(R).T
	return np.r_[np.c_[Rt, -np.dot(Rt, p)], [[0, 0, 0, 1]]]

# credit to mrcl
def se3ToVec(se3mat):
	return np.r_[[se3mat[2][1], se3mat[0][2], se3mat[1][0]],
	         [se3mat[0][3], se3mat[1][3], se3mat[2][3]]]

# credit to mrcl
def MatrixLog6(T):
	R, p = TransToRp(T)
	if NearZero(np.linalg.norm(R - np.eye(3))):
	    return np.r_[np.c_[np.zeros((3, 3)),
	                       [T[0][3], T[1][3], T[2][3]]],
	                 [[0, 0, 0, 0]]]
	else:
	    acosinput = (np.trace(R) - 1) / 2.0
	    if acosinput > 1:
	        acosinput = 1
	    elif acosinput < -1:
	        acosinput = -1
	    theta = np.arccos(acosinput)
	    omgmat = MatrixLog3(R)
	    return np.r_[np.c_[omgmat,
	                       np.dot(np.eye(3) - omgmat / 2.0 \
	                       + (1.0 / theta - 1.0 / np.tan(theta / 2.0) / 2) \
	                         * np.dot(omgmat,omgmat) / theta,[T[0][3],
	                                                          T[1][3],
	                                                          T[2][3]])],
	                 [[0, 0, 0, 0]]]

# credit to mcrl
def JacobianBody(Blist, thetalist):
	Jb = np.array(Blist).copy().astype(np.float)
	T = np.eye(4)
	for i in range(len(thetalist) - 2, -1, -1):
	    T = np.dot(T,MatrixExp6(VecTose3(np.array(Blist)[:, i + 1] \
	                                     * -thetalist[i + 1])))
	    Jb[:, i] = np.dot(Adjoint(T), np.array(Blist)[:, i])
	return Jb

# credit to mcrl
def FKinBody(M, Blist, thetalist):
	T = np.array(M)
	for i in range(len(thetalist)):
	    T = np.dot(T, MatrixExp6(VecTose3(np.array(Blist)[:, i] \
	                                      * thetalist[i])))
	return T

# credit to mcrl
def IKinBody(Blist, M, T, thetalist0, eomg, ev):
	thetalist = np.array(thetalist0).copy()
	i = 0
	maxiterations = 20
	Vb = se3ToVec(MatrixLog6(np.dot(TransInv(FKinBody(M, Blist, \
	                                                  thetalist)), T)))
	err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
	      or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
	while err and i < maxiterations:
	    thetalist = thetalist \
	                + np.dot(np.linalg.pinv(JacobianBody(Blist, \
	                                                     thetalist)), Vb)
	    i = i + 1
	    Vb \
	    = se3ToVec(MatrixLog6(np.dot(TransInv(FKinBody(M, Blist, \
	                                                   thetalist)), T)))
	    err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
	          or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
	return thetalist, not err

def control_input():
	# Define directional vector ('forward/backward' vector for linear motion with angle and magnitude)
	direcvecangle = ((np.pi)/180)*float(input('Angle off-norm for linear motion: ')) ## this should change along with input direction (e.g. from joystick)
	direcmagnitude = 1
	direcvecX = direcmagnitude*np.sin(direcvecangle)
	direcvecY = directmagnitude*np.cos(direcvecangle)
	direcvec = np.array([direcvecangle,direcvecX,direcvecY,direcmagnitude])
	return direcvec

def calculate_gait(direcvec,riseheight):
	halfstridelength = np.sqrt(direcvec[1]**2 + direcvec[2]**2) - (np.sqrt(direcvec[1]**2 + direcvec[2]**2)/4)
	cornerstridelength = (np.sqrt(direcvec[1]**2 + direcvec[2]**2)/8)
	cornerriselength = riseheight/8
	pureriselength = (riseheight/8)*2
	gaitparam = np.array(halfstridelength,cornerstridelength,cornerriselength,pureriselength)
	return gaitparam

def add_direcvec(T,direcvec,gaitparam):
	gait1 = np.array([np.cos(direcvec[0]),np.sin(direcvec[0]),0,np.cos(halfstridelength)],
						[-(np.sin(direcvec[0])),np.cos(direcvec[0]),0,np.sin(halfstridelength)],
						[0,0,1,0],
						[0,0,0,1])
	gait2 = np.array([],
					 [],
					 [],
					 [0,0,0,1])
	gait3 = np.array([],
					 [],
					 [],
					 [0,0,0,1])
	gait4 = np.array([],
					 [],
					 [],
					 [0,0,0,1])
	gait5 = np.array([],
					 [],
					 [],
					 [0,0,0,1])
	gait6 = np.array([],
					 [],
					 [],
					 [0,0,0,1])
	gait7 = np.array([],
					 [],
					 [],
					 [0,0,0,1])
	gait8 = np.array([],
					 [],
					 [],
					 [0,0,0,1])
	gait9 = np.array([],
					 [],
					 [],
					 [0,0,0,1])
	gait10 = np.array([],
					 [],
					 [],
					 [0,0,0,1])
	gait11 = np.array([],
					 [],
					 [],
					 [0,0,0,1])
	gait12 = np.array([],
					 [],
					 [],
					 [0,0,0,1])
	gait13 = np.array([],
					 [],
					 [],
					 [0,0,0,1])
	gait14 = np.array([],
					 [],
					 [],
					 [0,0,0,1])
	#xnew = T[0][3] + direcvec[1]
	#ynew = T[1][3] + direcvec[2]
	#T[0][3] = xnew
	#T[1][3] = ynew
	T1 = np.dot(T,Ttopoint)
	T2 = np.dot(T1,)
	Trajlist = [T1,T2,T3,T4,T5,T6,T7,T8,T9,T10,T11,T12,T13]
	return Traj

def generate_thetalist():
	thetalist = np.array([theta])

'''def workspace_trim():'''





'''def drive_servos():'''

def linear_motion():
	i = 0
	ii = 0
 	iii = 0
	iv = 0
	v = 0
	vi = 0
	vii = 0
	Mlist = np.array([[0],[0],[0],[0],[0],[0]])
	Tlist = np.array([[0],[0],[0],[0],[0],[0]])
	#reset_purestance()

	# compute home configurations for all legs
	while i < 5:
		Mlist[0][i] = homeconfig(thetalist[i],phinot,psinot)
		i += 1

	# compute forward kinematics in body for given home configurations
	while ii < 5:
		Tlist[0][ii] = FKinBody(Mlist[ii],Blist,thetalist)
		ii += 1

	'''put a check here'''

	# establish directional vector
	direcvec = control_input()
	gaitparam = calculate_gait(direcvec,riseheight)

	# add directional vector to EE
	while iii < 5:
		Tlist[iii] = add_direcvec(Tlist[iii],direcvec)
		iii += 1

	while iv < 5:
		thetanext,err = IKinBody(Blist, Mlist[iv], Tlist[iv], anglelistFull[0][iv], eomg, ev)
		anglelistFull[0][iv] = thetanext
		iv += 1

	while v < 5:
		#drive servos
		v  += 1

	while vi < 5:
		vi += 1


try:
	linear_motion()
except KeyboardInterrupt:
	#reset_purestance()
	print('\nExiting.')
	exit()
