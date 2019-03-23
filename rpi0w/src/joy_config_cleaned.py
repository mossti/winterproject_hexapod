#!/usr/bin/env python
import roslib
import rospy
import geometry_msgs.msg
import std_msgs.msg
from sensor_msgs.msg import Joy
from ServoPi import PWM
import time
import math

pwm = PWM(0x40)
pwm.set_pwm_freq(50)

ts = 0.05

low = 205
mid = 307
high = 410

def angle_convert(angle):
	pulse_range = (2.0 - 1.0)
	pw_per_deg = (pulse_range/181)
	pulse_width = int(((1.0) + (angle*(pw_per_deg)))*low)
	#print pulse_width
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

def callback(data):
	triangle = data.buttons[2]
	trialnum = 0
	num = 0
	numturns = 777
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.buttons)
		#print('\n(',trialnum,')')
		#print('\nTurn via rounded rectangular gait.','\nNOTE: safe values at: 90, 30, 90, 60')
		#print('\n------------------------------------------------------')
	hipmid = 90 #int(input('\nStance angle of hip: '))
	kneestance = 20 #int(input('\nMax. Stance angle of knee: '))
	kneerise = 160 #int(input('\nMax. rise angle of knee: '))
	stridelength = 100 #int(input('\nMax width (angle) of gait: '))
		#turn_direction = int(input('\nDirection of turn [1/-1]: '))
		#numturns = int(input('\nNumber of turns: '))
	velocity = 0.05 #float(input('\nVelocity: '))
		#direction = int(input('\nforward[0]/right[1]/backward[2]/left[3]/turn[4]: '))
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
	hip2 = hip1 - i*(halfcornerstride)
	knee2 = knee1 + halfcornerrise
	hip3 = hip2 - i*(halfcornerstride)
	knee3 = knee2 + halfcornerrise
	hip4 = hip3
	knee4 = knee3 + midheightrise
	hip5 = hip2
	knee5 = knee4 + halfcornerrise
	hip6 = hip1
	knee6 = knee5 + halfcornerrise
	hip7 = hip0
	knee7 = knee6
	hip8 = hip0 + i*(halfstancestride)
	knee8 = knee7
	hip9 = hip8 + i*(halfcornerstride)
	knee9 = knee5
	hip10 = hip9 + i*(halfcornerstride)
	knee10 = knee4
	hip11 = hip10
	knee11 = knee3
	hip12 = hip9
	knee12 = knee2
	hip13 = hip8
	knee13 = knee0
	hip14 = hip0
	knee14 = knee0

	# reverse gait plan
	hip0r = hipmid
	knee0r = kneestance
	hip1r = hipmid - j*(halfstancestride)
	knee1r = knee0r
	hip2r = hip1r - j*(halfcornerstride)
	knee2r = knee1r + halfcornerrise
	hip3r = hip2r - j*(halfcornerstride)
	knee3r = knee2r + halfcornerrise
	hip4r = hip3r
	knee4r = knee3r + midheightrise
	hip5r = hip2r
	knee5r = knee4r + halfcornerrise
	hip6r = hip1r
	knee6r = knee5r + halfcornerrise
	hip7r = hip0r
	knee7r = knee6r
	hip8r = hip0r + j*(halfstancestride)
	knee8r = knee7r
	hip9r = hip8r + j*(halfcornerstride)
	knee9r = knee5r
	hip10r = hip9r + j*(halfcornerstride)
	knee10r = knee4r
	hip11r = hip10r
	knee11r = knee3r
	hip12r = hip9r
	knee12r = knee2r
	hip13r = hip8r
	knee13r = knee0r
	hip14r = hip0r
	knee14r = knee0r

	# values for gait cycle when leg is on-angle of direction

	OAhip0x = hipmid
	OAknee0x = kneestance + 10
	OAhip1x = hipmid - i*(halfstancestride)
	OAknee1x = kneestance
	OAhip2x = OAhip1x - i*(halfcornerstride)
	OAknee2x = OAknee1x + halfcornerrise
	OAhip3x = OAhip2x - i*(halfcornerstride)
	OAknee3x = OAknee2x + halfcornerrise
	OAhip4x = OAhip3x
	OAknee4x = OAknee3x + midheightrise
	OAhip5x = OAhip2x
	OAknee5x = OAknee4x + halfcornerrise
	OAhip6x = OAhip1x
	OAknee6x = OAknee5x + halfcornerrise
	OAhip7x = OAhip0x
	OAknee7x = OAknee6x
	OAhip8x = OAhip0x + i*(halfstancestride)
	OAknee8x = OAknee7x
	OAhip9x = OAhip8x + i*(halfcornerstride)
	OAknee9x = OAknee5x
	OAhip10x = OAhip9x + i*(halfcornerstride)
	OAknee10x = OAknee4x
	OAhip11x = OAhip10x
	OAknee11x = OAknee3x
	OAhip12x = OAhip9x
	OAknee12x = OAknee2x
	OAhip13x = OAhip8x
	OAknee13x = OAknee0x

	OAhip0xr = hipmid
	OAknee0xr = kneestance + 10
	OAhip1xr = hipmid - j*(halfstancestride)
	OAknee1xr = kneestance
	OAhip2xr = OAhip1xr - j*(halfcornerstride)
	OAknee2xr = OAknee1xr + halfcornerrise
	OAhip3xr = OAhip2xr - j*(halfcornerstride)
	OAknee3xr = OAknee2xr + halfcornerrise
	OAhip4xr = OAhip3xr
	OAknee4xr = OAknee3xr + midheightrise
	OAhip5xr = OAhip2xr
	OAknee5xr = OAknee4xr + halfcornerrise
	OAhip6xr = OAhip1xr
	OAknee6xr = OAknee5xr + halfcornerrise
	OAhip7xr = OAhip0xr
	OAknee7xr = OAknee6xr
	OAhip8xr = OAhip0xr + j*(halfstancestride)
	OAknee8xr = OAknee7xr
	OAhip9xr = OAhip8xr + j*(halfcornerstride)
	OAknee9xr = OAknee5xr
	OAhip10xr = OAhip9xr + j*(halfcornerstride)
	OAknee10xr = OAknee4xr
	OAhip11xr = OAhip10xr
	OAknee11xr = OAknee3xr
	OAhip12xr = OAhip9xr
	OAknee12xr = OAknee2xr
	OAhip13xr = OAhip8xr
	OAknee13xr = OAknee0xr

	OAhip0v = hipmid
	OAknee0v = kneestance - 10
	OAhip1v = hipmid - i*(halfstancestride)
	OAknee1v = kneestance
	OAhip2v = OAhip1v - i*(halfcornerstride)
	OAknee2v = OAknee1v + halfcornerrise
	OAhip3v = OAhip2v - i*(halfcornerstride)
	OAknee3v = OAknee2v + halfcornerrise
	OAhip4v = OAhip3v
	OAknee4v = OAknee3v + midheightrise
	OAhip5v = OAhip2v
	OAknee5v = OAknee4v + halfcornerrise
	OAhip6v = OAhip1v
	OAknee6v = OAknee5v + halfcornerrise
	OAhip7v = OAhip0v
	OAknee7v = OAknee6v
	OAhip8v = OAhip0v + i*(halfstancestride)
	OAknee8v = OAknee7v
	OAhip9v = OAhip8v + i*(halfcornerstride)
	OAknee9v = OAknee5v
	OAhip10v = OAhip9v + i*(halfcornerstride)
	OAknee10v = OAknee4v
	OAhip11v = OAhip10v
	OAknee11v = OAknee3v
	OAhip12v = OAhip9v
	OAknee12v = OAknee2v
	OAhip13v = OAhip8v
	OAknee13v = OAknee0v

	OAhip0vr = hipmid
	OAknee0vr = kneestance - 10
	OAhip1vr = hipmid - j*(halfstancestride)
	OAknee1vr = kneestance
	OAhip2vr = OAhip1vr - j*(halfcornerstride)
	OAknee2vr = OAknee1vr + halfcornerrise
	OAhip3vr = OAhip2vr - j*(halfcornerstride)
	OAknee3vr = OAknee2vr + halfcornerrise
	OAhip4vr = OAhip3vr
	OAknee4vr = OAknee3vr + midheightrise
	OAhip5vr = OAhip2vr
	OAknee5vr = OAknee4vr + halfcornerrise
	OAhip6vr = OAhip1vr
	OAknee6vr = OAknee5vr + halfcornerrise
	OAhip7vr = OAhip0vr
	OAknee7vr = OAknee6vr
	OAhip8vr = OAhip0vr + j*(halfstancestride)
	OAknee8vr = OAknee7vr
	OAhip9vr = OAhip8vr + j*(halfcornerstride)
	OAknee9vr = OAknee5vr
	OAhip10vr = OAhip9vr + j*(halfcornerstride)
	OAknee10vr = OAknee4vr
	OAhip11vr = OAhip10vr
	OAknee11vr = OAknee3vr
	OAhip12vr = OAhip9vr
	OAknee12vr = OAknee2vr
	OAhip13vr = OAhip8vr
	OAknee13vr = OAknee0vr

	# turn values
	hip0t = hipmid
	knee0t = kneestance
	hip1t = hipmid - (halfstancestride)
	knee1t = knee0t
	hip2t = hip1t - (halfcornerstride)
	knee2t = knee1t + halfcornerrise
	hip3t = hip2t - (halfcornerstride)
	knee3t = knee2t + halfcornerrise
	hip4t = hip3t
	knee4t = knee3t + midheightrise
	hip5t = hip2t
	knee5t = knee4t + halfcornerrise
	hip6t = hip1t
	knee6t = knee5t + halfcornerrise
	hip7t = hip0t
	knee7t = knee6t
	hip8t = hip0t + (halfstancestride)
	knee8t = knee7t
	hip9t = hip8t + (halfcornerstride)
	knee9t = knee5t
	hip10t = hip9t + (halfcornerstride)
	knee10t = knee4t
	hip11t = hip10t
	knee11t = knee3t
	hip12t = hip9t
	knee12t = knee2t
	hip13t = hip8t
	knee13t = knee0t
	hip14t = hip0t
	knee14t = knee0t
	hip0tr = hipmid
	knee0tr = kneestance
		
	hip1tr = hipmid + (halfstancestride)
	knee1tr = knee0tr
	hip2tr = hip1tr + (halfcornerstride)
	knee2tr = knee1tr + halfcornerrise
	hip3tr = hip2tr + (halfcornerstride)
	knee3tr = knee2tr + halfcornerrise
	hip4tr = hip3tr
	knee4tr = knee3tr + midheightrise
	hip5tr = hip2tr
	knee5tr = knee4tr + halfcornerrise
	hip6tr = hip1tr
	knee6tr = knee5tr + halfcornerrise
	hip7tr = hip0tr
	knee7tr = knee6tr
	hip8tr = hip0tr - (halfstancestride)
	knee8tr = knee7tr
	hip9tr = hip8tr - (halfcornerstride)
	knee9tr = knee5tr
	hip10tr = hip9tr - (halfcornerstride)
	knee10tr = knee4tr
	hip11tr = hip10tr
	knee11tr = knee3tr
	hip12tr = hip9tr
	knee12tr = knee2tr
	hip13tr = hip8tr
	knee13tr = knee0tr
	hip14tr = hip0tr
	knee14tr = knee0tr
		
	### stance lists

	# normal gait
	hiplist0 = [hip0, hip1, hip1, hip1, hip1, hip1, hip1, hip1, hip1, hip1, hip2, hip3, hip4, hip5, hip6, hip7, hip8, hip9, hip10, hip11, hip12, hip13]
	kneelist0 = [knee0, knee1, knee1, knee1, knee1, knee1, knee1, knee1, knee1, knee1, knee2, knee3, knee4, knee5, knee6, knee7, knee8, knee9, knee10, knee11, knee12, knee13]
	hiplist1 = [hip3, hip4, hip5, hip6, hip7, hip8, hip9, hip10, hip11, hip12, hip13, hip0, hip1, hip1, hip1, hip1, hip1, hip1, hip1, hip1, hip1, hip2]
	kneelist1 = [knee3, knee4, knee5, knee6, knee7, knee8, knee9, knee10, knee11, knee12, knee13, knee0, knee1, knee1, knee1, knee1, knee1, knee1, knee1, knee1, knee1, knee2]
		
	hiplist0r = [hip0r, hip1r, hip1r, hip1r, hip1r, hip1r, hip1r, hip1r, hip1r, hip1r, hip2r, hip3r, hip4r, hip5r, hip6r, hip7r, hip8r, hip9r, hip10r, hip11r, hip12r, hip13r]
	kneelist0r = [knee0r, knee1r, knee1r, knee1r, knee1r, knee1r, knee1r, knee1r, knee1r, knee1r, knee2r, knee3r, knee4r, knee5r, knee6r, knee7r, knee8r, knee9r, knee10r, knee11r, knee12r, knee13r]
	hiplist1r = [hip3r, hip4r, hip5r, hip6r, hip7r, hip8r, hip9r, hip10r, hip11r, hip12r, hip13r, hip0r, hip1r, hip1r, hip1r, hip1r, hip1r, hip1r, hip1r, hip1r, hip1r, hip2r]
	kneelist1r = [knee3r, knee4r, knee5r, knee6r, knee7r, knee8r, knee9r, knee10r, knee11r, knee12r, knee13r, knee0r, knee1r, knee1r, knee1r, knee1r, knee1r, knee1r, knee1r, knee1r, knee1r, knee2r]
		
	# on-angle gait
	# push
	OAhiplist0x = [OAhip0x, OAhip1x, OAhip1x, OAhip1x, OAhip1x, OAhip1x, OAhip1x, OAhip1x, OAhip1x, OAhip1x, OAhip2x, OAhip3x, OAhip4x, OAhip5x, OAhip6x, OAhip7x, OAhip8x, OAhip9x, OAhip10x, OAhip11x, OAhip12x, OAhip13x]
	OAkneelist0x = [OAknee0x, OAknee1x, OAknee1x, OAknee1x, OAknee1x, OAknee1x, OAknee1x, OAknee1x, OAknee1x, OAknee1x, OAknee2x, OAknee3x, OAknee4x, OAknee5x, OAknee6x, OAknee7x, OAknee8x, OAknee9x, OAknee10x, OAknee11x, OAknee12x, OAknee13x]
	OAhiplist1x = [OAhip3x, OAhip4x, OAhip5x, OAhip6x, OAhip7x, OAhip8x, OAhip9x, OAhip10x, OAhip11x, OAhip12x, OAhip13x, OAhip0x, OAhip1x, OAhip1x, OAhip1x, OAhip1x, OAhip1x, OAhip1x, OAhip1x, OAhip1x, OAhip1x, OAhip2x]
	OAkneelist1x = [OAknee3x, OAknee4x, OAknee5x, OAknee6x, OAknee7x, OAknee8x, OAknee9x, OAknee10x, OAknee11x, OAknee12x, OAknee13x, OAknee0x, OAknee1x, OAknee1x, OAknee1x, OAknee1x, OAknee1x, OAknee1x, OAknee1x, OAknee1x, OAknee1x, OAknee2x]

	OAhiplist0xr = [OAhip0xr, OAhip1xr, OAhip1xr, OAhip1xr, OAhip1xr, OAhip1xr, OAhip1xr, OAhip1xr, OAhip1xr, OAhip1xr, OAhip2xr, OAhip3xr, OAhip4xr, OAhip5xr, OAhip6xr, OAhip7xr, OAhip8xr, OAhip9xr, OAhip10xr, OAhip11xr, OAhip12xr, OAhip13xr]
	OAkneelist0xr = [OAknee0xr, OAknee1xr, OAknee1xr, OAknee1xr, OAknee1xr, OAknee1xr, OAknee1xr, OAknee1xr, OAknee1xr, OAknee1xr, OAknee2xr, OAknee3xr, OAknee4xr, OAknee5xr, OAknee6xr, OAknee7xr, OAknee8xr, OAknee9xr, OAknee10xr, OAknee11xr, OAknee12xr, OAknee13xr]
	OAhiplist1xr = [OAhip3xr, OAhip4xr, OAhip5xr, OAhip6xr, OAhip7xr, OAhip8xr, OAhip9xr, OAhip10xr, OAhip11xr, OAhip12xr, OAhip13xr, OAhip0xr, OAhip1xr, OAhip1xr, OAhip1xr, OAhip1xr, OAhip1xr, OAhip1xr, OAhip1xr, OAhip1xr, OAhip1xr, OAhip2xr]
	OAkneelist1xr = [OAknee3xr, OAknee4xr, OAknee5xr, OAknee6xr, OAknee7xr, OAknee8xr, OAknee9xr, OAknee10xr, OAknee11xr, OAknee12xr, OAknee13xr, OAknee0xr, OAknee1xr, OAknee1xr, OAknee1xr, OAknee1xr, OAknee1xr, OAknee1xr, OAknee1xr, OAknee1xr, OAknee1xr, OAknee2xr]
		
	# pull
	OAhiplist0v = [OAhip0v, OAhip1v, OAhip1v, OAhip1v, OAhip1v, OAhip1v, OAhip1v, OAhip1v, OAhip1v, OAhip1v, OAhip2v, OAhip3v, OAhip4v, OAhip5v, OAhip6v, OAhip7v, OAhip8v, OAhip9v, OAhip10v, OAhip11v, OAhip12v, OAhip13v]
	OAkneelist0v = [OAknee0v, OAknee1v, OAknee1v, OAknee1v, OAknee1v, OAknee1v, OAknee1v, OAknee1v, OAknee1v, OAknee1v, OAknee2v, OAknee3v, OAknee4v, OAknee5v, OAknee6v, OAknee7v, OAknee8v, OAknee9v, OAknee10v, OAknee11v, OAknee12v, OAknee13v]
	OAhiplist1v = [OAhip3v, OAhip4v, OAhip5v, OAhip6v, OAhip7v, OAhip8v, OAhip9v, OAhip10v, OAhip11v, OAhip12v, OAhip13v, OAhip0v, OAhip1v, OAhip1v, OAhip1v, OAhip1v, OAhip1v, OAhip1v, OAhip1v, OAhip1v, OAhip1v, OAhip2v]
	OAkneelist1v = [OAknee3v, OAknee4v, OAknee5v, OAknee6v, OAknee7v, OAknee8v, OAknee9v, OAknee10v, OAknee11v, OAknee12v, OAknee13v, OAknee0v, OAknee1v, OAknee1v, OAknee1v, OAknee1v, OAknee1v, OAknee1v, OAknee1v, OAknee1v, OAknee1v, OAknee2v]

	OAhiplist0vr = [OAhip0vr, OAhip1vr, OAhip1vr, OAhip1vr, OAhip1vr, OAhip1vr, OAhip1vr, OAhip1vr, OAhip1vr, OAhip1vr, OAhip2vr, OAhip3vr, OAhip4vr, OAhip5vr, OAhip6vr, OAhip7vr, OAhip8vr, OAhip9vr, OAhip10vr, OAhip11vr, OAhip12vr, OAhip13vr]
	OAkneelist0vr = [OAknee0vr, OAknee1vr, OAknee1vr, OAknee1vr, OAknee1vr, OAknee1vr, OAknee1vr, OAknee1vr, OAknee1vr, OAknee1vr, OAknee2vr, OAknee3vr, OAknee4vr, OAknee5vr, OAknee6vr, OAknee7vr, OAknee8vr, OAknee9vr, OAknee10vr, OAknee11vr, OAknee12vr, OAknee13vr]
	OAhiplist1vr = [OAhip3vr, OAhip4vr, OAhip5vr, OAhip6vr, OAhip7vr, OAhip8vr, OAhip9vr, OAhip10vr, OAhip11vr, OAhip12vr, OAhip13vr, OAhip0vr, OAhip1vr, OAhip1vr, OAhip1vr, OAhip1vr, OAhip1vr, OAhip1vr, OAhip1vr, OAhip1vr, OAhip1vr, OAhip2vr]
	OAkneelist1vr = [OAknee3vr, OAknee4vr, OAknee5vr, OAknee6vr, OAknee7vr, OAknee8vr, OAknee9vr, OAknee10vr, OAknee11vr, OAknee12vr, OAknee13vr, OAknee0vr, OAknee1vr, OAknee1vr, OAknee1vr, OAknee1vr, OAknee1vr, OAknee1vr, OAknee1vr, OAknee1vr, OAknee1vr, OAknee2vr]

	# turn gaits
	hiplist0t = [hip14t,hip1t,hip1t,hip1t,hip1t,hip1t,hip1t,hip1t,hip1t,hip1t,hip2t,hip3t,hip4t,hip5t,hip6t,hip7t,hip8t,hip9t,hip10t,hip11t,hip12t,hip13t]
	kneelist0t = [knee14t,knee1t,knee1t,knee1t,knee1t,knee1t,knee1t,knee1t,knee1t,knee1t,knee2t,knee3t,knee4t,knee5t,knee6t,knee7t,knee8t,knee9t,knee10t,knee11t,knee12t,knee13t]
	hiplist1t = [hip3t,hip4t,hip5t,hip6t,hip7t,hip8t,hip9t,hip10t,hip11t,hip12t,hip13t,hip14t,hip1t,hip1t,hip1t,hip1t,hip1t,hip1t,hip1t,hip1t,hip1t,hip2t]
	kneelist1t = [knee3t,knee4t,knee5t,knee6t,knee7t,knee8t,knee9t,knee10t,knee11t,knee12t,knee13t,knee14t,knee1t,knee1t,knee1t,knee1t,knee1t,knee1t,knee1t,knee1t,knee1t,knee2t]

	hiplist0tr = [hip14tr,hip1tr,hip1tr,hip1tr,hip1tr,hip1tr,hip1tr,hip1tr,hip1tr,hip1tr,hip2tr,hip3tr,hip4tr,hip5tr,hip6tr,hip7tr,hip8tr,hip9tr,hip10tr,hip11tr,hip12tr,hip13tr]
	kneelist0tr = [knee14tr,knee1tr,knee1tr,knee1tr,knee1tr,knee1tr,knee1tr,knee1tr,knee1tr,knee1tr,knee2tr,knee3tr,knee4tr,knee5tr,knee6tr,knee7tr,knee8tr,knee9tr,knee10tr,knee11tr,knee12tr,knee13tr]
	hiplist1tr = [hip3tr,hip4tr,hip5tr,hip6tr,hip7tr,hip8tr,hip9tr,hip10tr,hip11tr,hip12tr,hip13tr,hip14tr,hip1tr,hip1tr,hip1tr,hip1tr,hip1tr,hip1tr,hip1tr,hip1tr,hip1tr,hip2tr]
	kneelist1tr = [knee3tr,knee4tr,knee5tr,knee6tr,knee7tr,knee8tr,knee9tr,knee10tr,knee11tr,knee12tr,knee13tr,knee14tr,knee1tr,knee1tr,knee1tr,knee1tr,knee1tr,knee1tr,knee1tr,knee1tr,knee1tr,knee2tr]



	# forward
	if data.buttons[2] == 1:

		if data.buttons[2] == 1: # and num<=numturns:
			j = 0
			while data.buttons[2] == 1 and data.buttons[2] != 0 and j<=21:
				rospy.sleep(ts)
				hip_move(0,(hiplist0[j],hiplist0[j],hiplist0r[j]))
				knee_move(0,(kneelist0[j],kneelist0[j],kneelist0r[j]))
				hip_move(1,(hiplist1[j],hiplist1r[j],hiplist1r[j]))
				knee_move(1,(kneelist1[j],kneelist1[j],kneelist1r[j]))
				time.sleep(velocity)
				j+=1
				#print data.buttons[2]
	# backward
	if data.buttons[0] == 1:

		if data.buttons[0] == 1: # and num<=numturns:
			j = 0
			while data.buttons[0] == 1 and data.buttons[0] != 0 and j <=21:
				rospy.sleep(ts)
				hip_move(0,(hiplist0r[j],hiplist0r[j],hiplist0[j]))
				knee_move(0,(kneelist0r[j],kneelist0r[j],kneelist0[j]))
				hip_move(1,(hiplist1r[j],hiplist1[j],hiplist1[j]))
				knee_move(1,(kneelist1r[j],kneelist1r[j],kneelist1[j]))
				time.sleep(velocity)
				j+=1
	# right
	if data.buttons[1] == 1:

		if data.buttons[1] == 1: # and num<=numturns:
			j = 0
			while data.buttons[1] == 1 and data.buttons[1] != 0 and j<=21:
				rospy.sleep(ts)
				hip_move(0,(hiplist0[j],hiplist0r[j],OAhiplist0v[j]))
				knee_move(0,(kneelist0[j],kneelist0r[j],OAkneelist0v[j]))
				hip_move(1,(OAhiplist1x[j],hiplist1r[j],hiplist1[j]))
				knee_move(1,(OAkneelist1x[j],kneelist1[j],kneelist1[j]))
				time.sleep(velocity)
				j+=1
	# left
	if data.buttons[3] == 1:

		if data.buttons[3] == 1: # and num<=numturns:
			j = 0
			#while data.buttons[3] == 1 and j<=21:
			while j in range(0,22) and data.buttons[3] == 1 and data.buttons[3] != 0:
				rospy.sleep(ts)
				hip_move(0,(hiplist0r[j],hiplist0[j],OAhiplist0x[j]))
				knee_move(0,(kneelist0r[j],kneelist0[j],OAkneelist0x[j]))
				hip_move(1,(OAhiplist1v[j],hiplist1[j],hiplist1r[j]))
				knee_move(1,(OAkneelist1v[j],kneelist1r[j],kneelist1r[j]))
				time.sleep(velocity)
				j+=1
	# rotate left
	if data.buttons[6] == 1:

		if data.buttons[6] == 1: # and num<=numturns:
			j = 0
			while data.buttons[6] == 1 and data.buttons[6] != 0 and j<=21:
				rospy.sleep(ts)
				hip_move(0,(hiplist0t[j],hiplist0t[j],hiplist0t[j]))
				knee_move(0,(kneelist0t[j],kneelist0t[j],kneelist0t[j]))
				hip_move(1,(hiplist1t[j],hiplist1t[j],hiplist1t[j]))
				knee_move(1,(kneelist1t[j],kneelist1t[j],kneelist1t[j]))
				time.sleep(velocity)
				j+=1
	# rotate right 
	if data.buttons[7] == 1:

		if data.buttons[7] == 1: # and num<=numturns:
			j = 0
			while data.buttons[7] == 1 and data.buttons[7] != 0 and j<=21:
				rospy.sleep(ts)
				hip_move(0,(hiplist0tr[j],hiplist0tr[j],hiplist0tr[j]))
				knee_move(0,(kneelist0tr[j],kneelist0tr[j],kneelist0tr[j]))
				hip_move(1,(hiplist1tr[j],hiplist1tr[j],hiplist1tr[j]))
				knee_move(1,(kneelist1tr[j],kneelist1tr[j],kneelist1tr[j]))
				time.sleep(velocity)
				j+=1

	# increment stance angle for knee (i.e. lower/raise height of body in stance)
	while data.buttons[13] == 1 and kneestance > 10 and kneestance < 50:
		kneestance += 5
		rospy.sleep(0.1)

	# increment stance angle for knee (i.e. lower/raise height of body in stance)
	while data.buttons[14] == 1 and kneestance > 10 and kneestance < 50:
		kneestance += (-5)
		rospy.sleep(0.1)

	# reset to specified stance
	if data.buttons[10] == 1:
		while data.buttons[10] == 1 and data.buttons[10] != 0:
			rospy.sleep(ts)
			hip_move(0,(hipmid,hipmid,hipmid))
			hip_move(1,(hipmid,hipmid,hipmid))
			knee_move(0,(kneestance,kneestance,kneestance))
			knee_move(1,(kneestance,kneestance,kneestance))
			time.sleep(velocity)

	trialnum+=1

def listener():
	rospy.init_node('listener', anonymous=True)
	jsinput = rospy.Subscriber('/joy', Joy, callback)
	rospy.spin()


if __name__ == '__main__':
	listener()
#except KeyboardInterrupt:
#	hip_move(0,(90,90,90))
#	hip_move(1,(90,90,90))
#	knee_move(0,(30,30,30))
#	knee_move(1,(30,30,30))
#	print('\nExiting.')
#	exit()
