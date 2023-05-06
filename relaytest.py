import RPi.GPIO as GPIO
import time


channel = 4

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(channel, GPIO.OUT)

from pynput import keyboard

def on_press(key):
    try:
        print('alphanumeric key {0} pressed'.format(
            key.char))
    except AttributeError:
        print('special key {0} pressed'.format(
            key))

def on_release(key):
    print('{0} released'.format(
        key))
    if key == keyboard.Key.esc:
        # Stop listener
        return False

 

# ...or, in a non-blocking fashion:
listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()






def laser_on(pin):
    GPIO.output(pin, GPIO.HIGH)  # Turn motor off

def laser_off(pin):
    GPIO.output(pin, GPIO.LOW)  # Turn motor on
    
def relay_off(pin):
    GPIO.output(pin, GPIO.HIGH)  # Turn motor off

def relay_on(pin):
    GPIO.output(pin, GPIO.LOW)  # Turn motor on



if __name__ == '__main__':
    try:
        motor_on(channel)
        time.sleep(1)
        motor_off(channel)
        time.sleep(10)
        GPIO.cleanup()
    except KeyboardInterrupt:
        GPIO.cleanup()
