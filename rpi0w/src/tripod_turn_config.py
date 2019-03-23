import rospy
from std_msgs.msg import String
from adafruit_servokit import ServoKit
import time
import math



kit = ServoKit(channels=16)

def hip_move(tripod,user_angle):
	ii=tripod
	while ii<=5:
		kit.servo[ii].angle = int(user_angle)
		ii+=2

def knee_move(tripod,user_angle):
	jj=tripod+6
	while jj<=11:
		kit.servo[jj].angle = int(user_angle)
		jj+=2

def turn_config():
	trialnum = 0
	num = 0
	rospy.init_node('listener',anonynous=True)
	rospy.Subscriber("/joy",float,callback)
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
		i=1

		#calculate halfstancestride and halfcornerstride
		halfstancestride = stridelength/3
		halfcornerstride = stridelength/12

		#calculate midheightrise and halfcornerrise
		midheightrise = kneerise/3
		halfcornerrise = kneerise/6
		
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

		
		#initialize gait arrays with path segments
		#hiplist0 = [hip1, hip2, hip3, hip4, hip5, hip6, hip7, hip8, hip9, hip10, hip11, hip12, hip13, hip14]
		#kneelist0 = [knee1, knee2, knee3, knee4, knee5, knee6, knee7, knee8, knee9, knee10, knee11, knee12, knee13, knee14]
		#hiplist1 = [hip7, hip8, hip9, hip10, hip11, hip12, hip13, hip14, hip1, hip2, hip3, hip4, hip5, hip6]
		#kneelist1 = [knee7, knee8, knee9, knee10, knee11, knee12, knee13, knee14, knee1, knee2, knee3, knee4, knee5, knee6]
		
		hiplist0 = [hip14,hip1,hip1,hip1,hip1,hip1,hip1,hip1,hip1,hip1,hip2,hip3,hip4,hip5,hip6,hip7,hip8,hip9,hip10,hip11,hip12,hip13]
		kneelist0 = [knee14,knee1,knee1,knee1,knee1,knee1,knee1,knee1,knee1,knee1,knee2,knee3,knee4,knee5,knee6,knee7,knee8,knee9,knee10,knee11,knee12,knee13]
		hiplist1 = [hip3,hip4,hip5,hip6,hip7,hip8,hip9,hip10,hip11,hip12,hip13,hip14,hip1,hip1,hip1,hip1,hip1,hip1,hip1,hip1,hip1,hip2]
		kneelist1 = [knee3,knee4,knee5,knee6,knee7,knee8,knee9,knee10,knee11,knee12,knee13,knee14,knee1,knee1,knee1,knee1,knee1,knee1,knee1,knee1,knee1,knee2]

		while num<=numturns:
			j = 0
			while j<=21:
				hip_move(0,hiplist0[j])
				knee_move(0,kneelist0[j])
				hip_move(1,hiplist1[j])
				knee_move(1,kneelist1[j])
				time.sleep(velocity)
				j+=1

		trialnum+=1
		rospy.spin()

try:
	turn_config()
except KeyboardInterrupt:
	hip_move(0,90)
	hip_move(1,90)
	knee_move(0,30)
	knee_move(1,30)
	print('\nExiting.')
	exit()
