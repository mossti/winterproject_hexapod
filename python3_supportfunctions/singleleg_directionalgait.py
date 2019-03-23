from adafruit_servokit import ServoKit
from mrclPY3 import Normalize,TransToRp,NearZero,TransInv,Adjoint,VecTose3,se3ToVec,VecToso3,so3ToVec,AxisAng3,MatrixExp6,MatrixLog6,JacobianBody,FKinBody,IKinBody
from transform_functions import homeconfig, control_input, calculate_gait, add_direcvec
from manual_set import manualset

import time
import numpy as np
#import rospy

kit = ServoKit(channels=16)

## Body screw axes
B_basetoHIP = np.array([0,0,1,0,-(0.0578104+0.04923984+0.0644906),0]) #np.array([0,0,1,0,-0.0578104,0]) #np.array([0, 0, 1, 0, 0, 0])
B_hiptoKNEE = np.array([0,0,1,0,-(0.04923984+0.0644906),0]) #np.array([0,0,1,0,-0.04923984-0.0578104,0]) #np.array([0, 0, 1, 0, 0.057803, 0])
B_kneetoEE = np.array([0,1,0,0,0,0.0737185]) #np.array([0,1,0,0,0.0737185,0]) #np.array([0, 1, 0, 0, 0, 0.04923984])
#B_list = np.array([B_basetoHIP, B_hiptoKNEE, B_kneetoEE]).T
B_list = np.array([B_basetoHIP, B_hiptoKNEE, B_kneetoEE]).T

## base to HIP (mechanically fixed)
theta0 = (30)*(np.pi/180)
theta1 = (90)*(np.pi/180)
theta2 = (150)*(np.pi/180)
theta3 = (210)*(np.pi/180)
theta4 = (270)*(np.pi/180)
theta5 = (330)*(np.pi/180)
thetalist = np.array([theta0,theta1,theta2,theta3,theta4,theta5])

## hip to KNEE (each leg 60 degrees from one another)
phinot = 0*((np.pi)/180)
## knee to LEG (30 degrees on servo ~ CHECK!!)
psinot = 0*((np.pi)/180) #-60*((np.pi)/180)

## angle list
anglelist_0 = np.array([theta0,phinot,psinot])

## max rise height (this will go to servo angle 90)
riseheight = 0*((np.pi)/180)

def directionalgait_singleleg(legnum):
    anglelist_0 = np.array([thetalist[legnum],phinot,-60*((np.pi)/180)])
    print('\noriginal angle list:\n')
    print(anglelist_0)
    homeconf = homeconfig(thetalist[legnum],phinot,psinot)
    print('\nhome configuration:\n')
    print(homeconf)
    current_tf = FKinBody(homeconf, B_list, anglelist_0)
    print('\ncurrent configuration:\n')
    print(current_tf)
    direcvec = control_input(thetalist[legnum],anglelist_0[1],anglelist_0[2])
    gaitparam = calculate_gait(direcvec,riseheight)
    next_tf = add_direcvec(current_tf,direcvec,gaitparam,anglelist_0[2])
    #anglelist_0 = np.array([thetalist[legnum],anglelist_0[1]+direcvec[1],anglelist_0[2]+direcvec[2]])
    print('\nnext configuration:\n')
    print(next_tf)
    anglelist_new,err = IKinBody(B_list, homeconf, next_tf, anglelist_0, 1.5,1.5)
    anglelist_0 = anglelist_new
    anglelist_servo = np.array([anglelist_0[1]*(180/np.pi)+90,anglelist_0[2]*(180/np.pi)+90])
    print('\nnew angle list:')
    print(anglelist_0)
    print(anglelist_servo)
    print(err)
    manualset()

try:
    legchoice = int(input('Which leg would you like to use? [0:5]:'))
    directionalgait_singleleg(legchoice)
except KeyboardInterrupt:
    print('\nExiting.')
    exit()
