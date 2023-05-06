from adafruit_servokit import ServoKit
import time
import cv2
import RPi.GPIO as GPIO
import dlib
from numpy import interp

xResolution = 426
yResolution = 256  #change this if wanted


kit = ServoKit(channels=16)

channelLaser = 21       #GPIO channel to control laser
relay = 12              #set relay GPIO channel
GPIO.setmode(GPIO.BCM)
GPIO.setup(channelLaser, GPIO.OUT)
GPIO.setup(relay, GPIO.OUT)


#global start_time
start_time = time.time()
print(start_time)

#nipple switches
rightNipple = 18  #right nipple
GPIO.setup(rightNipple, GPIO.IN, pull_up_down=GPIO.PUD_UP)
leftNipple = 17  #left nipple needs to be inversed
GPIO.setup(leftNipple, GPIO.IN, pull_up_down=GPIO.PUD_UP)


#Servos
armL = 4
armR = 5
xAxisServo = 14
yAxisServo = 15
retractServo = 2

#servo 4 left arm 110-170 closed-open           Left arm
#servo 5 right arm 95-10 closed-open            Right arm
#servo 14  0 right 90 is straingt 180 left      X-axis
#servo 15   0 up   60 is front      180 down               Y-axis
#servo 2 10 retracted 140 out

shotDone = 0
openVariable = 0
asleep = 0
shotcountdown = 0
        
def penis_in():
    global xAxisServo
    global yAxisServo
    global retractServo
    kit.servo[xAxisServo].angle = 75
    time.sleep(0.2)
    kit.servo[yAxisServo].angle = 60
    time.sleep(0.2)
    kit.servo[retractServo].angle = 15
    time.sleep(0.2)

def penis_out():
    global xAxisServo
    global yAxisServo
    global retractServo
    kit.servo[xAxisServo].angle = 75
    time.sleep(0.2)
    kit.servo[yAxisServo].angle = 60
    time.sleep(0.2)
    kit.servo[retractServo].angle = 140
    time.sleep(0.2)
    


def open_arms():  #open both arms of santa
    global armL
    global armR
    kit.servo[armL].angle = 170
    time.sleep(1)
    kit.servo[armR].angle = 10
    
    
def close_arms(): #close both arms of santa
    global armL
    global armR
    kit.servo[armR].angle = 95
    time.sleep(1)
    kit.servo[armL].angle = 110
    
        
def initialize_variables(servo):
    angle = float(input('Enter angle between 0 & 180: '))
    #kit.servo[servo].angle = angle
    return angle

def aim_servo(x,y):
    global xResolution, yResolution, minX, minY, maxX, maxY, xAxisServo, yAxisServo
    angleX = interp(x, [0,xResolution], [minX,maxX])
    angleY = interp(y, [0,yResolution], [minY,maxY])
    kit.servo[xAxisServo].angle = angleX
    kit.servo[yAxisServo].angle = angleY
    
    
def laser_on(pin):
    GPIO.output(pin, GPIO.HIGH)  # Turn motor off

def laser_off(pin):
    GPIO.output(pin, GPIO.LOW)  # Turn motor on
    
def relay_off(pin):
    GPIO.output(pin, GPIO.HIGH)  # Turn motor off

def relay_on(pin):
    GPIO.output(pin, GPIO.LOW)  # Turn motor on
    
#def open_jacket():

                                                        #to be written---------------------------------------------
#def close_jacket():


#Initialize a face cascade using the frontal face haar cascade provided with
#the OpenCV library
faceCascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

capture = cv2.VideoCapture(0)
capture.set(3,xResolution);
capture.set(4,yResolution);
#The deisred output width and height
#OUTPUT_SIZE_WIDTH = 775
#OUTPUT_SIZE_HEIGHT = 600
OUTPUT_SIZE_WIDTH = 426
OUTPUT_SIZE_HEIGHT = 240
#OUTPUT_SIZE_WIDTH = 256
#OUTPUT_SIZE_HEIGHT = 144
rectangleColor = (255,0,0)

def detectAndTrackLargestFace():
    #Open the first webcame device
    #capture = cv2.VideoCapture(0)
    #capture.set(3,426);
    #capture.set(4,240);
    
    #Create two opencv named windows
    #cv2.namedWindow("base-image", cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow("result-image", cv2.WINDOW_AUTOSIZE)

    #Position the windows next to eachother
    #cv2.moveWindow("base-image",0,100)
    cv2.moveWindow("result-image",400,100)

    #Start the window thread for the two windows we are using
    cv2.startWindowThread()

    #Create the tracker we will use
    tracker = dlib.correlation_tracker()

    #The variable we use to keep track of the fact whether we are
    #currently using the dlib tracker
    trackingFace = 0

    #The color of the rectangle we draw around the face
    #rectangleColor = (255,0,0)


    try:
        while True:
            #Retrieve the latest image from the webcam
            #rc,fullSizeBaseImage = capture.read()
            rc,baseImage = capture.read()
            #Resize the image to 320x240
            #baseImage = cv2.resize( fullSizeBaseImage, ( 320, 240))


            #Check if a key was pressed and if it was Q, then destroy all
            #opencv windows and exit the application
            pressedKey = cv2.waitKey(2)
            if pressedKey == ord('Q'):
                cv2.destroyAllWindows()
                exit(0)



            #Result image is the image we will show the user, which is a
            #combination of the original image from the webcam and the
            #overlayed rectangle for the largest face
            resultImage = baseImage.copy()






            #If we are not tracking a face, then try to detect one
            if not trackingFace:
                lost_face()


                #For the face detection, we need to make use of a gray
                #colored image so we will convert the baseImage to a
                #gray-based image
                gray = cv2.cvtColor(baseImage, cv2.COLOR_BGR2GRAY)
                #Now use the haar cascade detector to find all faces
                #in the image
                faces = faceCascade.detectMultiScale(gray, 1.2, 5)

                #In the console we can show that only now we are
                #using the detector for a face
                print("Using the cascade detector to detect face")


                #For now, we are only interested in the 'largest'
                #face, and we determine this based on the largest
                #area of the found rectangle. First initialize the
                #required variables to 0
                maxArea = 0
                x = 0
                y = 0
                w = 0
                h = 0


                #Loop over all faces and check if the area for this
                #face is the largest so far
                #We need to convert it to int here because of the
                #requirement of the dlib tracker. If we omit the cast to
                #int here, you will get cast errors since the detector
                #returns numpy.int32 and the tracker requires an int
                for (_x,_y,_w,_h) in faces:
                    if  _w*_h > maxArea:
                        x = int(_x)
                        y = int(_y)
                        w = int(_w)
                        h = int(_h)
                        maxArea = w*h

                #If one or more faces are found, initialize the tracker
                #on the largest face in the picture
                if maxArea > 0 :

                    #Initialize the tracker
                    leftX = int(x+0.5*w-7)
                    leftY = int(y+0.75*h-2)
                    rightX = int(x+0.5*w+10)
                    rightY = int(y+0.75*h+3)
                    mouthX = int(x+0.5*w)
                    mouthY = int(y+0.7*h)
                    
                    
                    #aim the servo
                    
                    
                    tracker.start_track(baseImage,
                                        dlib.rectangle( x,
                                                        y,
                                                        x+w,
                                                        y+h))

                    #Set the indicator variable such that we know the
                    #tracker is tracking a region in the image
                    trackingFace = 1

            #Check if the tracker is actively tracking a region in the image
            if trackingFace:

                #Update the tracker and request information about the
                #quality of the tracking update
                trackingQuality = tracker.update( baseImage )



                #If the tracking quality is good enough, determine the
                #updated position of the tracked region and draw the
                #rectangle
                if trackingQuality >= 8.75:
                    
                    rectangleColor = found_a_face_long()

                    tracked_position =  tracker.get_position()

                    t_x = int(tracked_position.left())
                    t_y = int(tracked_position.top())
                    t_w = int(tracked_position.width())
                    t_h = int(tracked_position.height())
                    cv2.rectangle(resultImage, (t_x, t_y),
                                                (t_x + t_w , t_y + t_h),
                                                rectangleColor ,2)
                    t_mouthX = int(t_x+0.5*t_w)
                    t_mouthY = int(t_y+0.2*t_h)
                    
                    
                    #aim the servo
                    global rightNipple
                    global leftNipple
                    global shotDone
                    global shotcountdown
                    valueRightNipple = GPIO.input(rightNipple)
                    inverseValueLeftNipple = GPIO.input(leftNipple)
                    if valueRightNipple == 1:
                        aim_servo(t_mouthX, t_mouthY)
                        #print("äiming")
                    if inverseValueLeftNipple == 1:
                        shotDone = 0
                        shotcountdown = 0
                    print(shotcountdown)
                    
                    #aim_servo(t_mouthX, t_mouthY)                            

                else:
                    #If the quality of the tracking update is not
                    #sufficient (e.g. the tracked region moved out of the
                    #screen) we stop the tracking of the face and in the
                    #next loop we will find the largest face in the image
                    #again
                    trackingFace = 0





            #Since we want to show something larger on the screen than the
            #original 320x240, we resize the image again
            #
            #Note that it would also be possible to keep the large version
            #of the baseimage and make the result image a copy of this large
            #base image and use the scaling factor to draw the rectangle
            #at the right coordinates.
            largeResult = cv2.resize(resultImage,
                                     (OUTPUT_SIZE_WIDTH,OUTPUT_SIZE_HEIGHT))

            #Finally, we want to show the images on the screen
            #cv2.imshow("base-image", baseImage)
            cv2.imshow("result-image", largeResult)




    #To ensure we can also deal with the user pressing Ctrl-C in the console
    #we have to check for the KeyboardInterrupt exception and destroy
    #all opencv windows and exit the application
    except KeyboardInterrupt as e:
        cv2.destroyAllWindows()
        GPIO.cleanup()
        exit(0)
        
relay_off(relay)

penis_in()
open_arms()
penis_out()


  
#turn on laser
laser_on(21)

#determine bounds of angles
while True:
    print("Min X initialisation")
    minX = initialize_variables(xAxisServo)
    retry = int(input('Want to retry? 1 or 0:) '))
    if retry == 0:
        break
while True:
    print("Max X initialisation")
    maxX = initialize_variables(xAxisServo)
    retry = int(input('Want to retry? 1 or 0:) '))
    if retry == 0:
        break
while True:
    print("Min Y initialisation")
    minY = initialize_variables(yAxisServo)
    retry = int(input('Want to retry? 1 or 0:) '))
    if retry == 0:
        break
while True:
    print("Max Y initialisation")
    maxY = initialize_variables(yAxisServo)
    retry = int(input('Want to retry? 1 or 0:) '))
    if retry == 0:
        break

        
#  interp(val, [0,256], [minX,maxX])
#  interp(val, [0,144], [minY,maxY])
laser_off(21)

penis_in()
close_arms()

counterFoundAFace = 0
antiCounterFoundAFace = 0

def found_a_face_long():
    global counterFoundAFace
    global antiCounterFoundAFace
    global start_time
    current_time = time.time()
    elapsed_time = current_time - start_time
    #print(elapsed_time)
    #print (counterFoundAFace)
    if elapsed_time > 2 or counterFoundAFace > 10:
        if counterFoundAFace>10:
            start_time = time.time()
            antiCounterFoundAFace = 0
            global rightNipple
            global leftNipple
            global shotDone
            global openVariable
            global shotcountdown
            global asleep
            asleep = 0
            valueRightNipple = GPIO.input(rightNipple)
            inverseValueLeftNipple = GPIO.input(leftNipple)
            #print(valueRightNipple,inverseValueLeftNipple)
            if valueRightNipple == 0 and inverseValueLeftNipple == 1 and openVariable == 0:
                openVariable = 1
                open_arms()
                time.sleep(0.5)
                penis_out()        
            if valueRightNipple == 1 and inverseValueLeftNipple == 0 and shotDone == 0:
                if shotcountdown > 50:
                    #shotDone = 1
                    relay_on(relay)
                    time.sleep(1)
                    relay_off(relay)
                    shotcountdown = 0
                shotcountdown = shotcountdown + 1
                
                   
            return((0,0,255))

            
            
            #Write here for if something needs to happen while detected for 5+seconds
            
            
            
            
        counterFoundAFace = counterFoundAFace+1
    #else:
    #    return((255,0,0))
        
def lost_face():
    global antiCounterFoundAFace
    global counterFoundAFace
    global start_time
    current_time = time.time()
    elapsed_time = current_time - start_time
    if elapsed_time > 1 or antiCounterFoundAFace > 1:
        if antiCounterFoundAFace>2:
            start_time = time.time()
            counterFoundAFace = 0
            print(elapsed_time)
            global rightNipple
            global leftNipple
            global asleep
            global openVariable
            valueRightNipple = GPIO.input(rightNipple)
            inverseValueLeftNipple = GPIO.input(leftNipple)
            if valueRightNipple == 0 and inverseValueLeftNipple == 1 and asleep == 0:
                penis_in()
                time.sleep(1)
                close_arms()
                asleep = 1
                openVariable = 0
                return((255,0,0))

        antiCounterFoundAFace = antiCounterFoundAFace+1


if __name__ == '__main__':
    detectAndTrackLargestFace()

#    if trackingFace == 0 and elapsed_time >20:
#        #close_jacket()
#        start_time = time.time()
