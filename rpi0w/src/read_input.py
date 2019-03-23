#!/usr/bin/env python

import roslib
import rospy
import std_msgs.msg
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from ServoPi import PWM
import time
import math

pwm = PWM(0x40)
pwm.set_pwm_freq(50)

ts = 0.2

low = 156
mid = 307
high = 458

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.buttons)
	if data.buttons[14] == 1:
		for i in range(1,13):
			while data.buttons[14] != 0:
				pwm.set_pwm(i, 0, mid)
				rospy.sleep(0.02)
	if data.buttons[15] == 1:
		for j in range(1,13):
			while data.buttons[15] != 0:
				pwm.set_pwm(j, 0, low)
				rospy.sleep(0.02)
	if data.buttons[13] == 1:
		for k in range(1,13):
			while data.buttons[13] != 0:
				pwm.set_pwm(k, 0, high)
				rospy.sleep(0.02)

def listener():

	rospy.init_node('listener',anonymous=True)
	jsinput = rospy.Subscriber('/joy', Joy, callback)
	
	print jsinput

	rospy.spin()

if __name__ == '__main__':
	listener()
