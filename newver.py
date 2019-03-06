import time
import numpy as np
import modern_robotics as mrcl

## base to HIP (mechanically fixed)
theta0 = (330)*(np.pi/180)
theta1 = (270)*(np.pi/180)
theta2 = (210)*(np.pi/180)
theta3 = (150)*(np.pi/180)
theta4 = (90)*(np.pi/180)
theta5 = (30)*(np.pi/180)
thetalist = np.array([theta0,theta1,theta2,theta3,theta4,theta5])

## hip to KNEE (each leg 60 degrees from one another)
phinot = 0*((np.pi)/180)

## knee to LEG (30 degrees on servo ~ CHECK!!)
psinot = 0*((np.pi)/180) #140-90-50*((np.pi)/180)

## max rise height (this will go to servo angle 90)

riseheight = 0*((np.pi)/180)

## screw axes

Ship = np.array{0, 0, 1, 0, 0, 0};
Sknee = np.array{0, 1, 0, 0, 0, 0.04923984};
Slist = Transpose({Ship, Sknee});

BhipTOknee = np.array(0, 0, 1, 0, 0.04923984 + 0.0737185, 0)
BkneeTOEE = np.array(0, 1, 0, 0, 0, 0.0737185)
Blist = Transpose(np.array({BhipTOknee, BkneeTOEE})

## home configuration for one leg:

legNORM = np.array({{-0.692965, 0.720971, 0, -0.128679},{-0.720971, -0.692965, 0, -0.00616296},{0,0,1,-0.0692727},{0,0,0,1}})
Mleg = np.array({{1, 0, 0, 0.04923984 + 0.0737185}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}});


## directional arrays

direcangle = np.array({{np.cos(omega),np.sin(omega),0,0},{-np.sin(omega),np.cos(omega),0,0},{0,0,1,0},{0,0,0,1}})
direcmagnitude = np.array({{1,0,0,magnitude},{0,1,0,0},{0,0,1,0},{0,0,0,1}})
direcvec = np.dot(direcangle.direcmagnitude)
direcvecP = np.array({direcvec[0][3],direcvec[1][3],direcvec[2][3]})


## transformation matrices for world frame and initial body

Tworldframe = np.array({{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}})
TbodyframeInit = Tworldframe

## home configuration from body to hip for each leg:

# fixed rotation from base frame to each hip joint
TbasecomROT0 = np.array({{np.sqrt(3)/2,1/2,0,0},{-1/2,np.sqrt(3)/2,0,0},{0,0,1,0},{0,0,0,1}})
TbasecomROT1 = np.array({{0,1,0,0},{-1,0,0,0},{0,0,1,0},{0,0,0,1}})
TbasecomROT2 = np.array({{-np.sqrt(3)/2,1/2,0,0},{-1/2,-np.sqrt(3)/2,0,0},{0,0,1,0},{0,0,0,1}})
TbasecomROT3 = np.array({{-np.sqrt(3)/2,-1/2,0,0},{1/2,-np.sqrt(3)/2,0,0},{0,0,1,0},{0,0,0,1}})
TbasecomROT4 = np.array({{0,-1,0,0},{1,0,0,0},{0,0,1,0},{0,0,0,1}})
TbasecomROT5 = np.array({{np.sqrt(3)/2,-1/2,0,0},{1/2,np.sqrt(3)/2,0,0},{0,0,1,0},{0,0,0,1}})

# translation from base frame to hip joint
TbasecomTOhipjoint = np.array({{1,0,0,0.0578104},{0,1,0,0},{0,0,1,-0.009611},{0,0,0,1}})

# transformation from base frame to each hip joint
TbaseTOhip0 = np.dot(TbasecomROT0,TbasecomTOhipjoint)
TbaseTOhip1 = np.dot(TbasecomROT1,TbasecomTOhipjoint)
TbaseTOhip2 = np.dot(TbasecomROT2,TbasecomTOhipjoint)
TbaseTOhip3 = np.dot(TbasecomROT3,TbasecomTOhipjoint)
TbaseTOhip4 = np.dot(TbasecomROT4,TbasecomTOhipjoint)
TbaseTOhip5= np.dot(TbasecomROT5,TbasecomTOhipjoint)

# rotation about knee joint
ThipjointROT0 = np.array({{np.cos(phi0),np.sin(phi0),0,0},{-np.sin(phi0),np.cos(phi0),0,0},{0,0,1,0},{0,0,0,1}})
ThipjointROT1 = np.array({{np.cos(phi1),np.sin(phi1),0,0},{-np.sin(phi1),np.cos(phi1),0,0},{0,0,1,0},{0,0,0,1}})
ThipjointROT2 = np.array({{np.cos(phi2),np.sin(phi2),0,0},{-np.sin(phi2),np.cos(phi2),0,0},{0,0,1,0},{0,0,0,1}})
ThipjointROT3 = np.array({{np.cos(phi3),np.sin(phi3),0,0},{-np.sin(phi3),np.cos(phi3),0,0},{0,0,1,0},{0,0,0,1}})
ThipjointROT4 = np.array({{np.cos(phi4),np.sin(phi4),0,0},{-np.sin(phi4),np.cos(phi4),0,0},{0,0,1,0},{0,0,0,1}})
ThipjointROT5 = np.array({{np.cos(phi5),np.sin(phi5),0,0},{-np.sin(phi5),np.cos(phi5),0,0},{0,0,1,0},{0,0,0,1}})

# transformation from hip joint to knee joint
ThipjointTOkneejoint = np.array({{1,0,0,0.04923984},{0,1,0,0},{0,0,1,0},{0,0,0,1}})

# rotation about knee joint
TkneejointROT0 = np.array({{np.cos(psi0),0,np.sin(psi0),0},{0,1,0,0},{-np.sin(psi0),0,np.cos(psi0),0},{0,0,0,1}})
TkneejointROT1 = np.array({{np.cos(psi1),0,np.sin(psi1),0},{0,1,0,0},{-np.sin(psi1),0,np.cos(psi1),0},{0,0,0,1}})
TkneejointROT2 = np.array({{np.cos(psi2),0,np.sin(psi2),0},{0,1,0,0},{-np.sin(psi2),0,np.cos(psi2),0},{0,0,0,1}})
TkneejointROT3 = np.array({{np.cos(psi3),0,np.sin(psi3),0},{0,1,0,0},{-np.sin(psi3),0,np.cos(psi3),0},{0,0,0,1}})
TkneejointROT4 = np.array({{np.cos(psi4),0,np.sin(psi4),0},{0,1,0,0},{-np.sin(psi4),0,np.cos(psi4),0},{0,0,0,1}})
TkneejointROT5 = np.array({{np.cos(psi5),0,np.sin(psi5),0},{0,1,0,0},{-np.sin(psi5),0,np.cos(psi5),0},{0,0,0,1}})

# translation from knee joint to end-effector
TkneejointTOEE = np.array({{1,0,0,0.0737185},{0,1,0,0},{0,0,1,0},{0,0,0,1}})

## transformation matrices for new body frame (b')
TbodyframeNew = TbodyframeInit
TbodyframeNew[0][3] = (TbodyframeNew[0][3] + direcvecPlot[0])
TbodyframeNew[1][3] = (TbodyframeNew[1][3] + direcvecPlot[1])
TbodyframeNew[2][3] = (TbodyframeNew[2][3] + direcvecPlot[2])
Tbprimeb = mrcl.TransInv(TbodyframeNew)
TbtoHIP = TbasecomTOhipjoint
TbprimeHIP = np.dot(Tbprimeb,TbtoHIP)

# representation of hip_ in (b')
TbodyTOhip0 = np.dot(TbasecomROT0,TbprimeHIP)
TbodyTOhip1 = np.dot(TbasecomROT1,TbprimeHIP)
TbodyTOhip2 = np.dot(TbasecomROT2,TbprimeHIP)
TbodyTOhip3 = np.dot(TbasecomROT3,TbprimeHIP)
TbodyTOhip4 = np.dot(TbasecomROT4,TbprimeHIP)
TbodyTOhip5 = np.dot(TbasecomROT5,TbprimeHIP)




def directionalgait_singleleg(legnum):
	anglelist_0 = np.array([thetalist[legnum],phinot,-60*((np.pi)/180)])