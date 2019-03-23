from adafruit_servokit import ServoKit
import time
import math

kit = ServoKit(channels=16)

def hip_move(tripod,user_angle_hiplist):
	ii=tripod
	kit.servo[ii].angle = int(user_angle_hiplist[0])
	kit.servo[ii+2].angle = int(user_angle_hiplist[1])
	kit.servo[ii+4].angle = int(user_angle_hiplist[2])


def knee_move(tripod,user_angle_kneelist):
	jj=tripod+6
	kit.servo[jj].angle = int(user_angle_kneelist[0])
	kit.servo[jj+2].angle = int(user_angle_kneelist[1])
	kit.servo[jj+4].angle = int(user_angle_kneelist[2])

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
		direction = int(input('\nDirection of turn [1/-1]: '))
		numturns = int(input('\nNumber of turns: '))
		velocity = float(input('\nVelocity: '))
		direction = input('\nforward or backward: ')
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

		


		#assign path values
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

		
		#initialize gait arrays with path segments
		#hiplist0 = [hip1, hip2, hip3, hip4, hip5, hip6, hip7, hip8, hip9, hip10, hip11, hip12, hip13, hip14]
		#kneelist0 = [knee1, knee2, knee3, knee4, knee5, knee6, knee7, knee8, knee9, knee10, knee11, knee12, knee13, knee14]
		#hiplist1 = [hip7, hip8, hip9, hip10, hip11, hip12, hip13, hip14, hip1, hip2, hip3, hip4, hip5, hip6]
		#kneelist1 = [knee7, knee8, knee9, knee10, knee11, knee12, knee13, knee14, knee1, knee2, knee3, knee4, knee5, knee6]
		
		if direction == 'forward':

			hiplist0 = [hip14,hip1,hip1,hip1,hip1,hip1,hip1,hip1,hip1,hip1,hip2,hip3,hip4,hip5,hip6,hip7,hip8,hip9,hip10,hip11,hip12,hip13]
			kneelist0 = [knee14,knee1,knee1,knee1,knee1,knee1,knee1,knee1,knee1,knee1,knee2,knee3,knee4,knee5,knee6,knee7,knee8,knee9,knee10,knee11,knee12,knee13]
			hiplist1 = [hip3,hip4,hip5,hip6,hip7,hip8,hip9,hip10,hip11,hip12,hip13,hip14,hip1,hip1,hip1,hip1,hip1,hip1,hip1,hip1,hip1,hip2]
			kneelist1 = [knee3,knee4,knee5,knee6,knee7,knee8,knee9,knee10,knee11,knee12,knee13,knee14,knee1,knee1,knee1,knee1,knee1,knee1,knee1,knee1,knee1,knee2]
		
			hiplist0r = [hip14r,hip1r,hip1r,hip1r,hip1r,hip2r,hip2r,hip2r,hip2r,hip2r,hip2r,hip3r,hip4r,hip5r,hip6r,hip7r,hip8r,hip9r,hip10r,hip11r,hip12r,hip13r]
			kneelist0r = [knee14r,knee1r,knee1r,knee1r,knee1r,knee2r,knee2r,knee2r,knee2r,knee2r,knee2r,knee3r,knee4r,knee5r,knee6r,knee7r,knee8r,knee9r,knee10r,knee11r,knee12r,knee13r]
			hiplist1r = [hip3r,hip4r,hip5r,hip6r,hip7r,hip8r,hip9r,hip10r,hip11r,hip12r,hip13r,hip14r,hip1r,hip1r,hip1r,hip1r,hip1r,hip1r,hip1r,hip1r,hip1r,hip2r]
			kneelist1r = [knee3r,knee4r,knee5r,knee6r,knee7r,knee8r,knee9r,knee10r,knee11r,knee12r,knee13r,knee14r,knee1r,knee1r,knee1r,knee1r,knee1r,knee1r,knee1r,knee1r,knee1r,knee2r]

			while num<=numturns:
				j = 0
				while j<=21:
					hip_move(0,(hiplist0[j],hiplist0[j],hiplist0r[j]))
					knee_move(0,(kneelist0[j],kneelist0[j],kneelist0r[j]))
					hip_move(1,(hiplist1[j],hiplist1r[j],hiplist1r[j]))
					knee_move(1,(kneelist1[j],kneelist1[j],kneelist1r[j]))
					time.sleep(velocity)
					j+=1


		if direction == 'backward':

			hiplist0 = [hip14,hip1,hip1,hip1,hip1,hip1,hip1,hip1,hip1,hip1,hip2,hip3,hip4,hip5,hip6,hip7,hip8,hip9,hip10,hip11,hip12,hip13]
			kneelist0 = [knee14,knee1,knee1,knee1,knee1,knee1,knee1,knee1,knee1,knee1,knee2,knee3,knee4,knee5,knee6,knee7,knee8,knee9,knee10,knee11,knee12,knee13]
			hiplist1 = [hip3,hip4,hip5,hip6,hip7,hip8,hip9,hip10,hip11,hip12,hip13,hip14,hip1,hip1,hip1,hip1,hip1,hip1,hip1,hip1,hip1,hip2]
			kneelist1 = [knee3,knee4,knee5,knee6,knee7,knee8,knee9,knee10,knee11,knee12,knee13,knee14,knee1,knee1,knee1,knee1,knee1,knee1,knee1,knee1,knee1,knee2]
		
			hiplist0r = [hip14r,hip1r,hip1r,hip1r,hip1r,hip1r,hip1r,hip1r,hip1r,hip1r,hip2r,hip3r,hip4r,hip5r,hip6r,hip7r,hip8r,hip9r,hip10r,hip11r,hip12r,hip13r]
			kneelist0r = [knee14r,knee1r,knee1r,knee1r,knee1r,knee1r,knee1r,knee1r,knee1r,knee1r,knee2r,knee3r,knee4r,knee5r,knee6r,knee7r,knee8r,knee9r,knee10r,knee11r,knee12r,knee13r]
			hiplist1r = [hip3r,hip4r,hip5r,hip6r,hip7r,hip8r,hip9r,hip10r,hip11r,hip12r,hip13r,hip14r,hip1r,hip1r,hip1r,hip1r,hip1r,hip1r,hip1r,hip1r,hip1r,hip2r]
			kneelist1r = [knee3r,knee4r,knee5r,knee6r,knee7r,knee8r,knee9r,knee10r,knee11r,knee12r,knee13r,knee14r,knee1r,knee1r,knee1r,knee1r,knee1r,knee1r,knee1r,knee1r,knee1r,knee2r]

			while num<=numturns:
				j = 0
				while j <=21:
					hip_move(0,(hiplist0r[j],hiplist0r[j],hiplist0[j]))
					knee_move(0,(kneelist0r[j],kneelist0r[j],kneelist0[j]))
					hip_move(1,(hiplist1r[j],hiplist1[j],hiplist1[j]))
					knee_move(1,(kneelist1r[j],kneelist1r[j],kneelist1[j]))
					time.sleep(velocity)
					j+=1

		trialnum+=1

try:
	turn_config()
except KeyboardInterrupt:
	hip_move(0,90)
	hip_move(1,90)
	knee_move(0,30)
	knee_move(1,30)
	print('\nExiting.')
	exit()
