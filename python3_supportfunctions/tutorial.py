from adafruit_servokit import ServoKit
import time
import math

kit = ServoKit(channels=16)

def servomanip():
	while True:
		kit.servo[0].angle=90
		kit.servo[1].angle=90
		time.sleep(1)
		kit.servo[0].angle=0
		kit.servo[1].angle=0
		time.sleep(1)
		kit.servo[0].angle=180
		kit.servo[1].angle=180
		time.sleep(1)
try:
	servomanip()
except KeyboardInterrupt:
	print('Exiting.')
	exit()

