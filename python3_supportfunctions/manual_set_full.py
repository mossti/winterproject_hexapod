from adafruit_servokit import ServoKit
import time

# define kit as 16 channel driver
kit = ServoKit(channels=16)

#function to set all hips to specific angle
def hip_move_circ(user_angle):
	i=0
	while i<=5:
		kit.servo[i].angle = int(user_angle)
		i+=1
#function to set all knees to specific angle
def knee_move_circ(user_angle):
	j=6
	while j<=11:
		kit.servo[j].angle = int(user_angle)
		j+=1

#function to set given tripod's (0 or 1) hip to specific angle
def hip_move_tri(tripod, user_angle):
	ii=tripod
	while ii<=5:
		kit.servo[ii].angle = int(user_angle)
		ii+=2

#function to set given tripod's (0 or 1) knee to specific angle
def knee_move_tri(tripod, user_angle):
	jj=tripod+6
	while jj<=11:
		kit.servo[jj].angle = int(user_angle)
		jj+=2

def manualset():
	setnum = 0;
	while True:
		# tracks number of times through program
		print('\n(',setnum,')')
		print('\n----------------------------')
		# gather user input
		singleormass = input('\nSingle servo or mass [1/0]: ')
		#manipulate single servo
		if singleormass=='1':
			servonum = input('\nWhich servo [0 -> 11]: ')
			angleval = input('\nServo angle: ')
			print('\nMoving servo',servonum,'to angle',angleval,'...')
			kit.servo[int(servonum)].angle = int(angleval)
		#manipulate multiple servos
		if singleormass=='0':
			#choose multiple servo pattern (circle or tripod)
			circortri = input('\nSet circle or alt. tripod [circ/tri]: ')
			#set circular pattern (hips/knees/all)
			if circortri=='circ':
				hka = input('\n[hip/knee/all]: ')
				angleval = input('\nServo angle: ')
				if hka=='hip':
					hip_move_circ(angleval)
				if hka=='knee':
					knee_move_circ(angleval)
				if hka=='all':
					hip_move_circ(angleval)
					knee_move_circ(angleval)
			#set tripod pattern (0/1/b),(hips/knees/all)
			if circortri=='tri':
				tri12b = input('\nTripod 0, 1, both [0/1/b]: ')
				hka = input('\n [hip/knee/all]: ')
				angleval = input('\nServo angle: ')
				if tri12b=='0':
					if hka=='hip':
						hip_move_tri(0,angleval)
					if hka=='knee':
						knee_move_tri(0,angleval)
					if hka=='all':
						hip_move_tri(0,angleval)
						knee_move_tri(0,angleval)
				if tri12b=='1':
					if hka=='hip':
						hip_move_tri(1,angleval)
					if hka=='knee':
						knee_move_tri(1,angleval)
					if hka=='all':
						hip_move_tri(1,angleval)
						knee_move_tri(1,angleval)
				if tri12b=='b':
					if hka=='hip':
						hip_move_tri(0,angleval)
						hip_move_tri(1,angleval)
					if hka=='knee':
						knee_move_tri(0,angleval)
						knee_move_tri(1,angleval)
					if hka=='all':
						hip_move_tri(0,angleval)
						knee_move_tri(0,angleval)
						hip_move_tri(1,angleval)
						knee_move_tri(1,angleval)
		
		# query for additional servo manual manip.
		cont = input('Manipulate other servo(s)? [y/n]: ')
		setnum+=1
		if cont=='n':
			print('Exiting.')
			exit()

try:
	manualset()
except KeyboardInterrupt:
	print('Exiting.')
	exit()
