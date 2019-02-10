from adafruit_servokit import ServoKit
import time
import math

kit = ServoKit(channels=16)

def knee_scuttle(direction,delay,user_angle_rise,user_angle_lower):
	if direction==1:
		ii=6
		while ii<=11:
			kit.servo[ii].angle = int(user_angle_rise)
			time.sleep(delay)
			kit.servo[ii].angle = int(user_angle_lower)
			ii+=1
	if direction==-1:
		ii=11
		while ii>=6:
			kit.servo[ii].angle = int(user_angle_rise)
			time.sleep(delay)
			kit.servo[ii].angle = int(user_angle_lower)
			ii-=1



def main():
	trialnum=0
	while True:
		print('--------------------------------')
		print('\n(',trialnum,')')
		print('\nScuttle functions (i.e. all legs acting with circular delay)')
		print('--------------------------------')
		user_angle_rise = int(input('Rise angle for knees: '))
		user_angle_lower = int(input('Return angle for knees: '))
		delay = float(input('Delay between leg motion: '))
		direction = int(input('Direction [1/-1]: '))
		
		knee_scuttle(direction,delay,user_angle_rise,user_angle_lower)

		cont = input('Continue? [y/n]: ')
		if cont=='n':
			print('\nExiting...')
			exit()
		
		trialnum+=1

try:
	main()
except KeyboardInterrupt:
	print('\nExiting...')
	exit()
