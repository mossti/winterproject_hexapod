from adafruit_servokit import ServoKit
import time
import numpy as np

kit = ServoKit(channels=16)

legiterate = 0
hipiterate = 0
kneeiterate = 0

leglist = np.zeros(6)
hiplist = np.zeros(6)
kneelist = np.zeros(6)
leg_exitcheck = 'y'
leg_add_count = 0
hip_exitcheck = 'y'
hip_add_count = 0
knee_exitcheck = 'y'
knee_add_count = 0

leg_to_add = 0
hip_to_add = 0
knee_to_add = 0

def reset():
	print('Resetting.')
	
	i = 0
	ii = 6

	legiterate = 0
	hipiterate = 0
	kneeiterate = 0

	leglist = np.zeros(6)
	hiplist = np.zeros(6)
	kneelist = np.zeros(6)

	leg_exitcheck = 'y'
	leg_add_count = 0
	hip_exitcheck = 'y'
	hip_add_count = 0
	knee_exitcheck = 'y'
	knee_add_count = 0
	
	while i <= 5:
		kit.servo[int(i)].angle = int(90)
		i += 1
	while ii <= 11:
		kit.servo[int(ii)].angle = int(30) 
		ii += 1
	return

def legmanip(leg,hipangle,kneeangle):
	kit.servo[int(leg)].angle = int(hipangle)
	kit.servo[int(leg+6)].angle = int(kneeangle)
	return


def jointloader():
	i = 0
	ii = 6

	legiterate = 0
	hipiterate = 0
	kneeiterate = 0

	leglist = np.zeros(6)
	hiplist = np.zeros(6)
	kneelist = np.zeros(6)

	leg_exitcheck = 'y'
	leg_add_count = 0
	hip_exitcheck = 'y'
	hip_add_count = 0
	knee_exitcheck = 'y'
	knee_add_count = 0
	reset()
	while True:
		reset()
		time.sleep(0.05)
		while leg_exitcheck == 'y' and leg_add_count < 6:
			Leg_to_add = input('\nPlease enter leg number [0->5]: ')
			leglist[leg_add_count] = leg_to_add
			leg_add_count += 1
			leg_exitcheck = input('\nManipulate another leg? [y/n]: ')
		while hip_exitcheck == 'y' and hip_add_count < leg_add_count:
			hip_to_add = input('\nPlease enter angle for hip '+str(hip_add_count)+': ')
			hiplist[hip_add_count] = hip_to_add
			hip_add_count += 1
			#hip_exitcheck = input('? [y/n]: ')
		while knee_exitcheck == 'y' and knee_add_count < leg_add_count:
			knee_to_add = input('\nPlease enter knee angle for knee'+str(knee_add_count)+': ')
			kneelist[knee_add_count] = knee_to_add
			knee_add_count += 1
			#knee_exitcheck = input('? [y/n]: ')
		while legiterate < leg_add_count:
			while hipiterate < hip_add_count and kneeiterate < knee_add_count:
				legmanip(leglist[legiterate],hiplist[hipiterate],kneelist[kneeiterate])
			leg_iterate += 1






try:
	reset()
	jointloader()
except KeyboardInterrupt:
	print('Exiting...')
	exit()
