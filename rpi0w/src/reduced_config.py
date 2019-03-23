#!/usr/bin/env python
import roslib
import rospy
import geometry_msgs.msg
import std_msgs.msg
from ServoPi import PWM
import time
import math

pwm = PWM(0x40)
pwm.set_pwm_freq(50)

ts = 0.2

low = 205
mid = 307
high = 410

def angle_convert(angle):
	pulse_range = (2.0 - 1.0)
	pw_per_deg = (pulse_range/181)
	pulse_width = int(((1.0) + (angle*(pw_per_deg)))*low)
	print pulse_width
	return pulse_width

def hip_move(tripod,user_angle_hiplist):
	angle_1 = angle_convert(user_angle_hiplist[0])
	angle_2 = angle_convert(user_angle_hiplist[1])
	angle_3 = angle_convert(user_angle_hiplist[2])
	ii=tripod+1
	pwm.set_pwm(ii, 0, angle_1)
	pwm.set_pwm((ii+2), 0, angle_2)
	pwm.set_pwm((ii+4), 0, angle_3)


def knee_move(tripod,user_angle_kneelist):
	angle_1 = angle_convert(user_angle_kneelist[0])
	angle_2 = angle_convert(user_angle_kneelist[1])
	angle_3 = angle_convert(user_angle_kneelist[2])
	jj=tripod+6+1
	pwm.set_pwm(jj, 0, angle_1)
	pwm.set_pwm((jj+2), 0, angle_2)
	pwm.set_pwm((jj+4), 0, angle_3)

def turn_config():
	trialnum = 0
	num = 0
	while True:
		print('\n(',trialnum,')')
		print('\nTurn via rounded rectangular gait.','\nNOTE: safe values at: 90, 30, 90, 60')
		print('\n------------------------------------------------------')
		hipmid = int(input('\nStance angle of hip: '))
		kneestance = int(input('\nMax. Stance angle of knee: '))
		kneerise = int(input('\nMax. rise angle of knee: '))
		stridelength = int(input('\nMax width (angle) of gait: '))
		turn_direction = int(input('\nDirection of turn [1/-1]: '))
		numturns = int(input('\nNumber of turns: '))
		velocity = float(input('\nVelocity: '))
		direction = int(input('\nforward[0]/right[1]/backward[2]/left[3]/turn[4]: '))
		i=1
		j=-1

		#calculate halfstancestride and halfcornerstride
		halfstancestride = stridelength/3
		halfcornerstride = stridelength/12

		#calculate midheightrise and halfcornerrise
		midheightrise = kneerise/3
		halfcornerrise = kneerise/6
		

		hip0 = hipmid
		knee0 = kneestance
		hip1 = hipmid - i*(halfstancestride)
		knee1 = knee0
		hip2 = hip0
		knee2 = kneerise
		hip3 = hipmid + i*(halfstancestride)
		knee3 = knee0

		
		hip0r = hipmid
		knee0r = kneestance
		hip1r = hipmid - j*(halfstancestride)
		knee1r = knee0r
		hip2r = hip0r
		knee2r = kneerise
		hip3r = hipmid + j*(halfstancestride)
		knee3r = knee0r

		# values for gait cycle when leg is on-angle of direction
		OAhip0 = hipmid
		OAknee0 = kneestance + 10
		OAhip1 = hipmid - 0.5*i*(halfstancestride)
		OAknee1 = kneestance #OAknee0
		OAhip2 = hipmid - i*(halfstancestride)
		OAknee2 = kneestance #OAknee0
		OAhip3 = hipmid - 0.5*i*(halfstancestride)
		OAknee3 = kneerise
		OAhip4 = OAhip0
		OAknee4 = kneestance #OAknee0
		OAhip5 = hipmid + 0.5*i*(halfstancestride)
		OAknee5 = kneestance
		OAhip6 = hipmid + i*(halfstancestride)
		OAknee6 = kneestance #OAknee0
		OAhip7 = hipmid + 0.5*i*(halfstancestride)
		OAknee7 = kneerise #OAknee0
		
		OAhip0r = hipmid
		OAknee0r = kneestance - 10
		OAhip1r = hipmid - 0.5*j*(halfstancestride)
		OAknee1r = kneestance #OAknee0r
		OAhip2r = hipmid - j*(halfstancestride)
		OAknee2r = kneestance #OAknee0r
		OAhip3r = hipmid - 0.5*j*(halfstancestride)
		OAknee3r = kneerise
		OAhip4r = OAhip0r
		OAknee4r = kneestance #OAknee0r
		OAhip5r = hipmid + 0.5*j*(halfstancestride)
		OAknee5r = kneestance
		OAhip6r = hipmid + j*(halfstancestride)
		OAknee6r = kneestance #OAknee0r
		OAhip7r = hipmid + 0.5*j*(halfstancestride)
		OAknee7r = kneerise #OAknee0r

		#initialize gait arrays with path segments
		#hiplist0 = [hip1, hip2, hip3, hip4, hip5, hip6, hip7, hip8, hip9, hip10, hip11, hip12, hip13, hip14]
		#kneelist0 = [knee1, knee2, knee3, knee4, knee5, knee6, knee7, knee8, knee9, knee10, knee11, knee12, knee13, knee14]
		#hiplist1 = [hip7, hip8, hip9, hip10, hip11, hip12, hip13, hip14, hip1, hip2, hip3, hip4, hip5, hip6]
		#kneelist1 = [knee7, knee8, knee9, knee10, knee11, knee12, knee13, knee14, knee1, knee2, knee3, knee4, knee5, knee6]
		
		# turn values
		hip0t = hipmid
		knee0t = kneestance
		
		hip1t = hipmid - turn_direction*(halfstancestride)
		knee1t = knee0t

		hip2t = hip1t - turn_direction*(halfcornerstride)
		knee2t = knee1t + halfcornerrise

		hip3t = hip2t - turn_direction*(halfcornerstride)
		knee3t = knee2t + halfcornerrise

		hip4t = hip3t
		knee4t = knee3t + midheightrise

 		hip5t = hip2t
		knee5t = knee4t + halfcornerrise
		
		hip6t = hip1t
		knee6t = knee5t + halfcornerrise

		hip7t = hip0t
		knee7t = knee6t

		hip8t = hip0t + turn_direction*(halfstancestride)
		knee8t = knee7t

		hip9t = hip8t + turn_direction*(halfcornerstride)
		knee9t = knee5t

		hip10t = hip9t + turn_direction*(halfcornerstride)
		knee10t = knee4t

		hip11t = hip10t
		knee11t = knee3t

		hip12t = hip9t
		knee12t = knee2t

		hip13t = hip8t
		knee13t = knee0t

		hip14t = hip0t
		knee14t = knee0t
		
		if direction == 0:

			hiplist0 = [hip0,hip1,hip2,hip3]
			kneelist0 = [knee0,knee1,knee2,knee3]
			hiplist1 = [hip2,hip3,hip0,hip1]
			kneelist1 = [knee2,knee3,knee0,knee1]
		
			hiplist0r = [hip0r,hip1r,hip2r,hip3r]
			kneelist0r = [knee0r,knee1r,knee2r,knee3r]
			hiplist1r = [hip2r,hip3r,hip0r,hip1r]
			kneelist1r = [knee2r,knee3r,knee0r,knee1r]

			while num<=numturns:
				j = 0
				while j<=3:
					hip_move(0,(hiplist0[j],hiplist0[j],(int(hiplist0r[j]))))
					knee_move(0,(kneelist0[j],kneelist0[j],kneelist0r[j]))
					hip_move(1,((int(hiplist1[j])),hiplist1r[j],hiplist1r[j]))
					knee_move(1,(kneelist1[j],kneelist1[j],kneelist1r[j]))
					time.sleep(velocity)
					j+=1


		if direction == 2:


			hiplist0 = [hip0,hip1,hip2,hip3]
			kneelist0 = [knee0,knee1,knee2,knee3]
			hiplist1 = [hip2,hip3,hip0,hip1]
			kneelist1 = [knee2,knee3,knee0,knee1]
		
			hiplist0r = [hip0r,hip1r,hip2r,hip3r]
			kneelist0r = [knee0r,knee1r,knee2r,knee3r]
			hiplist1r = [hip2r,hip3r,hip0r,hip1r]
			kneelist1r = [knee2r,knee3r,knee0r,knee1r]

			while num<=numturns:
				j = 0
				while j <=3:
					hip_move(0,(hiplist0r[j],hiplist0r[j],hiplist0[j]))
					knee_move(0,(kneelist0r[j],kneelist0r[j],kneelist0[j]))
					hip_move(1,(hiplist1r[j],hiplist1[j],hiplist1[j]))
					knee_move(1,(kneelist1r[j],kneelist1r[j],kneelist1[j]))
					time.sleep(velocity)
					j+=1

		if direction == 1:


			hiplist0 = [hip0,(0.5*(hip0+hip1)),hip1,(0.5*(hip1+hip2)),hip2,(0.5*(hip2+hip3)),hip3,(0.5*(hip3+hip0))]
			kneelist0 = [knee0,(0.5*(knee0+knee1)),knee1,(0.5*(knee1+knee2)),knee2,(0.5*(knee2+knee3)),knee3,(0.5*(knee3+knee0))]
			hiplist1 = [hip2,(0.5*(hip2+hip3)),hip3,(0.5*(hip3+hip0)),hip0,(0.5*(hip0+hip1)),hip1,(0.5*(hip1+hip2))]
			kneelist1 = [knee2,(0.5*(knee2+knee3)),knee3,(0.5*(knee3+knee0)),knee0,(0.5*(knee0+knee1)),knee1,(0.5*(knee1+knee2))]
		
			hiplist0r = [hip0r,(0.5*(hip0r+hip1r)),hip1r,(0.5*(hip1r+hip2r)),hip2r,(0.5*(hip2r+hip3r)),hip3r,(0.5*(hip3r+hip0r))]
			kneelist0r = [knee0r,(0.5*(knee0r+knee1r)),knee1r,(0.5*(knee1r+knee2r)),knee2r,(0.5*(knee2r+knee3r)),knee3r,(0.5*(knee3r+knee0r))]
			hiplist1r = [hip2r,(0.5*(hip2r+hip3r)),hip3r,(0.5*(hip3r+hip0r)),hip0r,(0.5*(hip0r+hip1r)),hip1r,(0.5*(hip1r+hip2r))]
			kneelist1r = [knee2r,(0.5*(knee2r+knee3r)),knee3r,(0.5*(knee3r+knee0r)),knee0r,(0.5*(knee0r+knee1r)),knee1r,(0.5*(knee1r+knee2r))]

			OAhiplist0 = [OAhip0,OAhip1,OAhip2,OAhip3,OAhip4,OAhip5,OAhip6,OAhip7]
			OAkneelist0 = [OAknee0,OAknee1,OAknee2,OAknee3,OAknee4,OAknee5,OAknee6,OAknee7]
			OAhiplist1 = [OAhip4,OAhip5,OAhip6,OAhip7,OAhip0,OAhip1,OAhip2,OAhip3]
			OAkneelist1 = [OAknee4,OAknee5,OAknee6,OAknee7,OAknee0,OAknee1,OAknee2,OAknee3]
		
			OAhiplist0r = [OAhip0r,OAhip1r,OAhip2r,OAhip3r,OAhip4r,OAhip5r,OAhip6r,OAhip7r]
			OAkneelist0r = [OAknee0r,OAknee1r,OAknee2r,OAknee3r,OAknee4r,OAknee5r,OAknee6r,OAknee7r]
			OAhiplist1r = [OAhip4r,OAhip5r,OAhip6r,OAhip7r,OAhip0r,OAhip1r,OAhip2r,OAhip3r]
			OAkneelist1r = [OAknee4r,OAknee5r,OAknee6r,OAknee7r,OAknee0r,OAknee1r,OAknee2r,OAknee3r]

			while num<=numturns:
				j = 0
				while j<=7:
					hip_move(0,(hiplist0[j],hiplist0r[j],OAhiplist0[j]))
					knee_move(0,(kneelist0[j],kneelist0r[j],OAkneelist0[j]))
					hip_move(1,(OAhiplist1[j],hiplist1r[j],hiplist1[j]))
					knee_move(1,(OAkneelist1[j],kneelist1[j],kneelist1[j]))
					time.sleep(velocity)
					j+=1


		if direction == 3:


			hiplist0 = [hip0,(0.5*(hip0+hip1)),hip1,(0.5*(hip1+hip2)),hip2,(0.5*(hip2+hip3)),hip3,(0.5*(hip3+hip0))]
			kneelist0 = [knee0,(0.5*(knee0+knee1)),knee1,(0.5*(knee1+knee2)),knee2,(0.5*(knee2+knee3)),knee3,(0.5*(knee3+knee0))]
			hiplist1 = [hip2,(0.5*(hip2+hip3)),hip3,(0.5*(hip3+hip0)),hip0,(0.5*(hip0+hip1)),hip1,(0.5*(hip1+hip2))]
			kneelist1 = [knee2,(0.5*(knee2+knee3)),knee3,(0.5*(knee3+knee0)),knee0,(0.5*(knee0+knee1)),knee1,(0.5*(knee1+knee2))]
		
			hiplist0r = [hip0r,(0.5*(hip0r+hip1r)),hip1r,(0.5*(hip1r+hip2r)),hip2r,(0.5*(hip2r+hip3r)),hip3r,(0.5*(hip3r+hip0r))]
			kneelist0r = [knee0r,(0.5*(knee0r+knee1r)),knee1r,(0.5*(knee1r+knee2r)),knee2r,(0.5*(knee2r+knee3r)),knee3r,(0.5*(knee3r+knee0r))]
			hiplist1r = [hip2r,(0.5*(hip2r+hip3r)),hip3r,(0.5*(hip3r+hip0r)),hip0r,(0.5*(hip0r+hip1r)),hip1r,(0.5*(hip1r+hip2r))]
			kneelist1r = [knee2r,(0.5*(knee2r+knee3r)),knee3r,(0.5*(knee3r+knee0r)),knee0r,(0.5*(knee0r+knee1r)),knee1r,(0.5*(knee1r+knee2r))]

			OAhiplist0 = [OAhip0,OAhip1,OAhip2,OAhip3,OAhip4,OAhip5,OAhip6,OAhip7]
			OAkneelist0 = [OAknee0,OAknee1,OAknee2,OAknee3,OAknee4,OAknee5,OAknee6,OAknee7]
			OAhiplist1 = [OAhip4,OAhip5,OAhip6,OAhip7,OAhip0,OAhip1,OAhip2,OAhip3]
			OAkneelist1 = [OAknee4,OAknee5,OAknee6,OAknee7,OAknee0,OAknee1,OAknee2,OAknee3]
		
			OAhiplist0r = [OAhip0r,OAhip1r,OAhip2r,OAhip3r,OAhip4r,OAhip5r,OAhip6r,OAhip7r]
			OAkneelist0r = [OAknee0r,OAknee1r,OAknee2r,OAknee3r,OAknee4r,OAknee5r,OAknee6r,OAknee7r]
			OAhiplist1r = [OAhip4r,OAhip5r,OAhip6r,OAhip7r,OAhip0r,OAhip1r,OAhip2r,OAhip3r]
			OAkneelist1r = [OAknee4r,OAknee5r,OAknee6r,OAknee7r,OAknee0r,OAknee1r,OAknee2r,OAknee3r]

			while num<=numturns:
				j = 0
				while j<=7:
					hip_move(0,(hiplist0r[j],hiplist0[j],OAhiplist0[j]))
					knee_move(0,(kneelist0r[j],kneelist0[j],OAkneelist0[j]))
					hip_move(1,(OAhiplist1[j],hiplist1[j],hiplist1r[j]))
					knee_move(1,(OAkneelist1[j],kneelist1r[j],kneelist1r[j]))
					time.sleep(velocity)
					j+=1

		if direction == 4:
			hiplist0t = [hip14t,hip1t,hip1t,hip1t,hip1t,hip1t,hip1t,hip1t,hip1t,hip1t,hip2t,hip3t,hip4t,hip5t,hip6t,hip7t,hip8t,hip9t,hip10t,hip11t,hip12t,hip13t]
			kneelist0t = [knee14t,knee1t,knee1t,knee1t,knee1t,knee1t,knee1t,knee1t,knee1t,knee1t,knee2t,knee3t,knee4t,knee5t,knee6t,knee7t,knee8t,knee9t,knee10t,knee11t,knee12t,knee13t]
			hiplist1t = [hip3t,hip4t,hip5t,hip6t,hip7t,hip8t,hip9t,hip10t,hip11t,hip12t,hip13t,hip14t,hip1t,hip1t,hip1t,hip1t,hip1t,hip1t,hip1t,hip1t,hip1t,hip2t]
			kneelist1t = [knee3t,knee4t,knee5t,knee6t,knee7t,knee8t,knee9t,knee10t,knee11t,knee12t,knee13t,knee14t,knee1t,knee1t,knee1t,knee1t,knee1t,knee1t,knee1t,knee1t,knee1t,knee2t]

			while num<=numturns:
				j = 0
				while j<=21:
					hip_move(0,(hiplist0t[j],hiplist0t[j],hiplist0t[j]))
					knee_move(0,(kneelist0t[j],kneelist0t[j],kneelist0t[j]))
					hip_move(1,(hiplist1t[j],hiplist1t[j],hiplist1t[j]))
					knee_move(1,(kneelist1t[j],kneelist1t[j],kneelist1t[j]))
					time.sleep(velocity)
					j+=1


		trialnum+=1

try:
	turn_config()
except KeyboardInterrupt:
	hip_move(0,(90,90,90))
	hip_move(1,(90,90,90))
	knee_move(0,(30,30,30))
	knee_move(1,(30,30,30))
	print('\nExiting.')
	exit()
