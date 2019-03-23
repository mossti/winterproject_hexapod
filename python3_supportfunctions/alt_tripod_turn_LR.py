from adafruit_servokit import ServoKit
import time
import math

kit = ServoKit(channels=16)

#function to set given tripod's (0 or 1) hip to specific angle
def hip_move(tripod, user_angle, speed):
    i=tripod
    while i<=5:
            kit.servo[i].angle = int(speed*user_angle)
            i += 2

#function to set given tripod's (0 or 1) knee to specific angle
def knee_move(tripod, user_angle, speed):
    j=tripod+6
    while j<=11:
            kit.servo[j].angle = int(speed*user_angle)
            j += 2

#function to turn left
def alt_tripod_left():

#function to turn right
def alt_tripod_right():

#main function; allows user to specify turn direction, velocity, and duration
def main():
    while True:
        direction = input('\nSelect direction of rotation [L/R]: \n')
        speed = input('\nSelect velocity: \n')
        duration = input('\nSelect # seconds to turn: \n')

try:
    main()
except KeyboardInterrupt:
    print('Exiting...')
    exit()
