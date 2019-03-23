from adafruit_servokit import ServoKit
from mrclPY3 import FKinBody
import time
import numpy as np

def homeconfig(theta,phi,psi):
    T_basetoHIP = np.array([[np.cos(theta), np.sin(theta), 0, 0.0578104],[-(np.sin(theta)), np.cos(theta), 0, 0],[0, 0, 1, -0.009611],[0, 0, 0, 1]])
    T_hiptoKNEE = np.array([[np.cos(phi), np.sin(phi), 0, 0.04923984],[-(np.sin(phi)), np.cos(phi), 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]])
    T_kneetoEE = np.array([[np.cos(psi), 0, np.sin(psi), 0.0644906],[0, 1, 0, 0],[-(np.sin(psi)), 0, np.cos(psi), -0.0357124],[0, 0, 0, 1]])
    T_basetoKNEE = np.dot(T_basetoHIP,T_hiptoKNEE)
    T_basetoEE = np.dot(T_basetoKNEE,T_kneetoEE)
    return T_basetoEE

def control_input(theta,phi,psi):
    direcvecangle = ((np.pi)/180)*(float(input('Angle off-norm for linear motion: '))-90)
    direcmagnitude = 1
    omega = (theta + phi + 90*(np.pi/180) - 2*direcvecangle) % 360
    direcvecX = direcmagnitude*np.cos(omega)
    direcvecY = direcmagnitude*np.sin(omega)
    direcvec = np.array([direcvecangle,direcvecX,direcvecY,direcmagnitude])
    return direcvec

def calculate_gait(direcvec,riseheight):
    halfstridelength = np.sqrt(direcvec[1]**2 + direcvec[2]**2) - (np.sqrt(direcvec[1]**2 + direcvec[2]**2)/4)
    cornerstridelength = (np.sqrt(direcvec[1]**2 + direcvec[2]**2)/8)
    cornerriselength = riseheight/8
    pureriselength = (riseheight/8)*2
    gaitparam = np.array([halfstridelength,cornerstridelength,cornerriselength,pureriselength])
    return gaitparam

def add_direcvec(T,direcvec,gaitparam,psi):
    #Tgait1 = np.array([[np.cos(-psi),0,np.sin(-psi),direcvec[1]],[0,1,0,0],[-(np.sin(-psi)),0,np.cos(psi),-0.045],[0,0,0,1]]) #check z val
    xnew = T[0][3] + direcvec[1]
    ynew = T[1][3] + direcvec[2]
    T[0][3] = xnew
    T[1][3] = ynew
    Tnew = T
    #Tnew = np.dot(T,Tgait1)
    return Tnew
