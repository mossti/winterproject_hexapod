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

# function for assigning all new joint angles
def newAnglesAssign(newphi0,newphi1,newphi2,newphi3,newphi4,newphi5,newpsi0,newpsi1,newpsi2,newpsi3,newpsi4,newpsi5):
	# new angles
	newPhi0 = newphi0*(np.pi/180)
	newPhi1 = newphi1*(np.pi/180)
	newPhi2 = newphi2*(np.pi/180)
	newPhi3 = newphi3*(np.pi/180)
	newPhi4 = newphi4*(np.pi/180)
	newPhi5 = newphi5*(np.pi/180)
	newPsi0 = newpsi0*(np.pi/180)
	newPsi1 = newpsi1*(np.pi/180)
	newPsi2 = newpsi2*(np.pi/180)
	newPsi3 = newpsi3*(np.pi/180)
	newPsi4 = newpsi4*(np.pi/180)
	newPsi5 = newpsi5*(np.pi/180)
	
	# store new angles in list
	AngleList0 = np.array({newphi0,newPsi0})
	AngleList1 = np.array({newphi1,newPsi1})
	AngleList2 = np.array({newphi2,newPsi2})
	AngleList3 = np.array({newphi3,newPsi3})
	AngleList4 = np.array({newphi4,newPsi4})
	AngleList5 = np.array({newphi5,newPsi5})
	
	return AngleList0,AngleList1,AngleList2,AngleList3,AngleList4,AngleList5

# function for performing FK in space for new angles
def FKallSpace(aList0,aList1,aList2,aList3,aList4,aList5):
	newPos0 = mrcl.FKinSpace(Mleg,Slist,aList0)
	newPos1 = mrcl.FKinSpace(Mleg,Slist,aList1)
	newPos2 = mrcl.FKinSpace(Mleg,Slist,aList2)
	newPos3 = mrcl.FKinSpace(Mleg,Slist,aList3)
	newPos4 = mrcl.FKinSpace(Mleg,Slist,aList4)
	newPos5 = mrcl.FKinSpace(Mleg,Slist,aList5)
	
	return newPos0,newPos1,newPos2,newPos3,newPos4,newPos5
	
# function which converts FK's EE position from the hip frame to the (b') frame
def convertFKtoBodyFrame(newP0,newP1,newP2,newP3,newP4,newP5):
	newPos0b = np.dot(TbodyTOhip0,newP0)
	newPos1b = np.dot(TbodyTOhip1,newP1)
	newPos2b = np.dot(TbodyTOhip2,newP2)
	newPos3b = np.dot(TbodyTOhip3,newP3)
	newPos4b = np.dot(TbodyTOhip4,newP4)
	newPos5b = np.dot(TbodyTOhip5,newP5)
	
	return newPos0b,newPos1b,newPos2b,newPos3b,newPos4b,newPos5b

# function which subtracts the directional control from EE in (b') frame
def calcStep(newP0b,newP1b,newP2b,newP3b,newP4b,newP5b,direcvecPLOT):

	newPos0bq = newP0b
	newPos0bq[0][3] = (newP0b[0][3] - (1*direcvecPLOT[0])
	newPos0bq[1][3] = (newP0b[1][3] - (1*direcvecPLOT[1])
	newPos0bq[2][3] = (newP0b[2][3] - (1*direcvecPLOT[2])
	
	newPos1bq = newP1b
	newPos1bq[0][3] = (newP1b[0][3] - (1*direcvecPLOT[0])
	newPos1bq[1][3] = (newP1b[1][3] - (1*direcvecPLOT[1])
	newPos1bq[2][3] = (newP1b[2][3] - (1*direcvecPLOT[2])
	
	newPos2bq = newP2b
	newPos2bq[0][3] = (newP2b[0][3] - (1*direcvecPLOT[0])
	newPos2bq[1][3] = (newP2b[1][3] - (1*direcvecPLOT[1])
	newPos2bq[2][3] = (newP2b[2][3] - (1*direcvecPLOT[2])
	
	newPos3bq = newP3b
	newPos3bq[0][3] = (newP3b[0][3] - (1*direcvecPLOT[0])
	newPos3bq[1][3] = (newP3b[1][3] - (1*direcvecPLOT[1])
	newPos3bq[2][3] = (newP3b[2][3] - (1*direcvecPLOT[2])
	
	newPos4bq = newP4b
	newPos4bq[0][3] = (newP4b[0][3] - (1*direcvecPLOT[0])
	newPos4bq[1][3] = (newP4b[1][3] - (1*direcvecPLOT[1])
	newPos4bq[2][3] = (newP4b[2][3] - (1*direcvecPLOT[2])
	
	newPos5bq = newP5b
	newPos5bq[0][3] = (newP5b[0][3] - (1*direcvecPLOT[0])
	newPos5bq[1][3] = (newP5b[1][3] - (1*direcvecPLOT[1])
	newPos5bq[2][3] = (newP5b[2][3] - (1*direcvecPLOT[2])
	
	return newPos0bq,newPos1bq,newPos2bq,newPos3bq,newPos4bq,newPos5bq

# convert end-effector from (b') frame to hip frame for IK
def convertPointBprimeFORIK(newP0bq,newP1bq,newP2bq,newP3bq,newP4bq,newP5bq):
	
	Tpinleg0 = mrcl.TransInv(TbodyTOhip0).newP0bq
	Tpinleg1 = mrcl.TransInv(TbodyTOhip1).newP1bq
	Tpinleg2 = mrcl.TransInv(TbodyTOhip2).newP2bq
	Tpinleg3 = mrcl.TransInv(TbodyTOhip3).newP3bq
	Tpinleg4 = mrcl.TransInv(TbodyTOhip4).newP4bq
	Tpinleg5 = mrcl.TransInv(TbodyTOhip5).newP5bq
	
	return Tpinleg0, Tpinleg1, Tpinleg2, Tpinleg3, Tpinleg4, Tpinleg5

# IK in space for all Tb',ee
def IKallSpace(Tpl0,Tpl1,Tpl2,Tpl3,Tpl4,Tpl5,aList0,aList1,aList2,aList3,aList4,aList5):
	
	newAngles0 = mrcl.IKinSpace(Slist,Mleg,Tpl0,aList0,0.01,0.001)
	newAngles1 = mrcl.IKinSpace(Slist,Mleg,Tpl1,aList1,0.01,0.001)
	newAngles2 = mrcl.IKinSpace(Slist,Mleg,Tpl2,aList2,0.01,0.001)
	newAngles3 = mrcl.IKinSpace(Slist,Mleg,Tpl3,aList3,0.01,0.001)
	newAngles4 = mrcl.IKinSpace(Slist,Mleg,Tpl4,aList4,0.01,0.001)
	newAngles5 = mrcl.IKinSpace(Slist,Mleg,Tpl5,aList5,0.01,0.001)

	return newAngles0,newAngles1,newAngles2,newAngles3,newAngles4,newAngles5
	
def main():
	aList0,aList1,aList2,aList3,aList4,aList5 = newAnglesAssign(0,0,0,0,0,0,-100,-100,-100,-100,-100,-100)
	FKleg0,FKleg1,FKleg2,FKleg3,FKleg4,FKleg5 = FKallSpace(aList0,aList1,aList2,aList3,aList4,aList5)
	FKbprime0,FKbprime1,FKbprime2,FKbprime3,FKbprime4,FKbprime5 = convertFKtoBodyFrame(FKleg0,FKleg1,FKleg2,FKleg3,FKleg4,FKleg5)
	newPbpq0,newPbpq1,newPbpq2,newPbpq3,newPbpq4,newPbpq5 = calcStep(FKbprime0,FKbprime1,FKbprime2,FKbprime3,FKbprime4,FKbprime5,direcvecPLOT)
	Tpinleg0, Tpinleg1, Tpinleg2, Tpinleg3, Tpinleg4, Tpinleg5 = convertPointBprimeFORIK(newPbpq0,newPbpq1,newPbpq2,newPbpq3,newPbpq4,newPbpq5)
	newaList0,newaList1,newaList2,newaList3,newaList4,newaList5 = IKallSpace(Tpinleg0, Tpinleg1, Tpinleg2, Tpinleg3, Tpinleg4, Tpinleg5,aList0,aList1,aList2,aList3,aList4,aList5)
	
try:
	main()
except KeyboardInterrupt:
	print('\nExiting.')
	exit()
	
	
	