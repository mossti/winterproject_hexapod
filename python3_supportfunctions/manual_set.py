from adafruit_servokit import ServoKit
import time

# define kit as 16 channel driver
kit = ServoKit(channels=16)

def manualset():
	while True:
		# gather user input
		servonum = input('Which servo: ')
		angleval = input('Servo angle: ')
		
		# move specified servo to specified angle
		print('Moving servo', servonum, 'to angle', angleval)
		kit.servo[int(servonum)].angle = int(angleval)
		
		# query for additional servo manual manip.
		cont = input('Change another servo? [y/n]')
		if cont=='n':
			print('Exiting.')
			exit()

#try:
#	manualset()
#except KeyboardInterrupt:
#	print('Exiting.')
#	exit()
