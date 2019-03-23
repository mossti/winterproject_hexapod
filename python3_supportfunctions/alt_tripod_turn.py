from adafruit_servokit import ServoKit
import time
import math

kit = ServoKit(channels=16)

#function to set given tripod's (0 or 1) hip to specific angle
def hip_move(tripod, user_angle):
    i=tripod
    while i<=5:
            kit.servo[i].angle = int(user_angle)
            i += 2

#function to set given tripod's (0 or 1) knee to specific angle
def knee_move(tripod, user_angle):
    j=tripod+6
    while j<=11:
            kit.servo[j].angle = int(user_angle)
            j += 2

def alt_tripod():
    while True:

    #start on tripod 0 (options: 0 or 1)
    i = 0 #hip joint index
    j = 6 #knee joint index

    #move tripod 0 hips
    print('Moving tripod 0 hips...')
    hip_move(0, 90)
    time.sleep(1)
    hip_move(0,45)
    time.sleep(1)
    hip_move(0,90)
    time.sleep(1)
    hip_move(0,135)
    time.sleep(1)
    hip_move(0,90)
    time.sleep(1)

try:
    alt_tripod()
except KeyboardInterrupt:
    print('Exiting.')
    exit()
