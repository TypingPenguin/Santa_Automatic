from adafruit_servokit import ServoKit
import time
import cv2
import RPi.GPIO as GPIO
import dlib
from numpy import interp


kit = ServoKit(channels=16)


def initialize_variables(servo):
    angle = float(input('Enter angle between 0 & 180: '))
    kit.servo[servo].angle = angle
    return angle


while True:
    print("Min X initialisation")
    minX = initialize_variables(2)
    #retry = int(input('Want to retry? 1 or 0:) '))
    #if retry == 0:
    #    break
    
    
#servo 4 left arm 110-170 closed-open           Left arm
#servo 5 right arm 95-10 closed-open            Right arm
#servo 14  0 right 90 is straingt 180 left      X-axis
#servo 15   0 up   60 is front      180 down               Y-axis
#servo 2 10 retracted 140 out
