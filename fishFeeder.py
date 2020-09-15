# -*- coding: utf-8 -*-
"""
Created on Tue Sep 15 15:05:15 2020

@author: HP
"""

import datetime
import RPi.GPIO as GPIO
from time import sleep
import time

import cv2
import numpy as np
from imutils.video import VideoStream
import imutils

######################SET UP SERVO##########################
now = datetime.datetime.now()

feedStartHour = 0
feedStartMinute = 20

feedEndHour = 0
feedEndMinute = 50

feedTime = now.replace(hour=feedStartHour, minute=feedStartMinute, second=0, microsecond=0)
feedEndTime = now.replace(hour=feedEndHour, minute=feedEndMinute, second=0, microsecond=0)

GPIO.setmode(GPIO.BOARD)

GPIO.setup(3, GPIO.OUT)
pwm=GPIO.PWM(3, 50)

pwm.start(0)

feedTimeStarted = False

def SetAngle(angle):
    duty = angle / 18 + 2
    GPIO.output(3, True)
    pwm.ChangeDutyCycle(duty)
    sleep(1)
    GPIO.output(3, False)
    pwm.ChangeDutyCycle(0)

SetAngle(0)

#################SET UP CAMERA######################
fishCountThreshold = 2

usingPiCamera = True

frameSize = (640, 480)


vs = VideoStream(src=1, usePiCamera=usingPiCamera, resolution=frameSize,framerate=32).start()

time.sleep(2.0)

timeCheck = time.time()

frame1 = vs.read()
frame2 = vs.read()
print(frame1.shape)



while(True):


    now = datetime.datetime.now()

    print(now)

    print("time "+str(feedEndTime))

    feedTime = now.replace(hour=feedStartHour, minute=feedStartMinute, second=0, microsecond=0)
    feedEndTime = now.replace(hour=feedEndHour, minute=feedEndMinute, second=0, microsecond=0)



    if now > feedTime and now < feedEndTime:
        print("true")

        
        frame = vs.read()

        if not usingPiCamera:
                    frame = imutils.resize(frame, width=frameSize[0])

        
        cv2.imshow('orig', frame)
        key = cv2.waitKey(1) & 0xFF

        diff = cv2.absdiff(frame1, frame2)
        

        gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
        

        blur = cv2.GaussianBlur(gray, (5,5), 0)
        

        _, thresh = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
        

        dilated = cv2.dilate(thresh, None, iterations=3)
       

        _, contours, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        

        fishCount = 0

        for contour in contours:
                (x, y, w, h) = cv2.boundingRect(contour)

                if cv2.contourArea(contour) < 10000:
                            continue

                fishCount=fishCount+1
                cv2.rectangle(frame1, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(frame1, "Status: {}".format('Movement & fishCount'+ str(fishCount)), (10, 20), cv2.FONT_HERSHEY_SIMPLEX,
                                    1, (0, 0, 255), 3)

                
                if(feedTimeStarted == False and fishCount>=fishCountThreshold):
                    SetAngle(180)
                    feedTimeStarted = True



        image = cv2.resize(frame1, (1280,720))
        cv2.imshow("feed", frame1)
        frame1 = frame2
        frame2 = vs.read()

        if cv2.waitKey(40) == 27:
                    break
        if key == ord("q"):
                    break

        print(1/(time.time() - timeCheck))
        timeCheck = time.time()



    else:
        print("false")
        if feedTimeStarted:
            feedTimeStarted = False
            SetAngle(0)

