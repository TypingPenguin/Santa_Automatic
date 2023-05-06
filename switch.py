import RPi.GPIO as GPIO
import time


GPIO.setmode(GPIO.BCM) 
rightNipple = 18  #right nipple
GPIO.setup(rightNipple, GPIO.IN, pull_up_down=GPIO.PUD_UP)
leftNipple = 17  #left nipple
GPIO.setup(leftNipple, GPIO.IN, pull_up_down=GPIO.PUD_UP)




while True:
    try:
        print(GPIO.input(rightNipple))
        time.sleep(1)
    except KeyboardInterrupt:
        GPIO.cleanup()
