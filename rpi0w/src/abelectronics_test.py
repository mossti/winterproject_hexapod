# heavily inspired by https://www.abelectronics.co.uk/kb/article/1078/generating-a-pwm-signal

from ServoPi import PWM
import time
pwm = PWM(0x40)

pwm.set_pwm_freq(50)


ts = 1

#output_enable()

low = 205
mid = 307
high = 410

i = 1


def angle_convert(angle):
	pulse_range = (2.0 - 1.0)
	pw_per_deg = (pulse_range/181)
	pulse_width = int((1.0 + (pw_per_deg*angle))*low)
	print pulse_width
	return pulse_width

def three_angles(a1, a2, a3):
	angle1 = angle_convert(a1)
	angle2 = angle_convert(a2)
	angle3 = angle_convert(a3)
	
	servochoice = int(input('Servo to cycle: '))
	
	pwm.set_pwm(servochoice, 0, angle1)
	time.sleep(ts)
	pwm.set_pwm(servochoice, 0, angle2)
	time.sleep(ts)
	pwm.set_pwm(servochoice, 0, angle3)
	time.sleep(ts)

def servo_test():
	i = 1
	while i < 13:

		pwm.set_pwm(i, 0, mid)

		time.sleep(ts)

		pwm.set_pwm(i, 0, low)

		time.sleep(ts)

		pwm.set_pwm(i, 0, mid)

		time.sleep(ts)

		pwm.set_pwm(i, 0, high)

		time.sleep(ts)

		pwm.set_pwm(i, 0, mid)

		time.sleep(ts)

		i += 1


def main():
	test = int(input('servotest[0]/3angles[1]: '))
	if test == 0:
		servo_test()
	if test ==1:
		ang1 = int(input('First angle: '))
		ang2 = int(input('Second angle: '))
		ang3 = int(input('Third angle: '))
		three_angles(ang1, ang2, ang3)


if __name__=='__main__':
	main()
