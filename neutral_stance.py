from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

def neutral_stance():
	while True:
		user_angle = input('Angle for all legs: ')
		i = 0
		while i<=12:
			kit.servo[i].angle = int(user_angle)
			i += 1


try:
	neutral_stance()
except KeyboardInterrupt:
	print('Exiting.')
	exit()
