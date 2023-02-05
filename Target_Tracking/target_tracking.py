import numpy as np
from imutils.video import VideoStream
import argparse
import imutils
import time
import cv2
import sys
from scipy.special import comb

vs = VideoStream(src=0).start()

frameList = []
numFrames = 0


while True:
    # grab the frame from the threaded video stream and resize it to have a maximum width of 1000 pixels
    frame = vs.read()
    frame = cv2.flip(frame, 1)
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
     
    lower_green = np.array([50,100,50])
    upper_green = np.array([70,255,255])
 
    # preparing the mask to overlay
    mask = cv2.inRange(hsv, lower_green, upper_green)
     
    # The black region in the mask has the value of 0,
    # so when multiplied with original image removes all non-blue regions
    result = cv2.bitwise_and(frame, frame, mask = mask)
    
    # convert image to grayscale image
    gray_image = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
     
    # convert the grayscale image to binary image
    ret,thresh = cv2.threshold(gray_image,127,255,0)
    
    # calculate moments of binary image
    M = cv2.moments(thresh)

    cX = int(M["m10"] / (M["m00"]+1))
    cY = int(M["m01"] / (M["m00"]+1))
    

    cv2.circle(frame,center=(cX, cY), radius=2, color=(255, 0, 0), thickness=4)

    cv2.imshow("Frame", frame)
    
    numFrames += 1
    cv2.imshow('frame', frame)
    #cv2.imshow('mask', mask)
    #cv2.imshow('result', result)
    #cv2.imshow('gray_image',gray_image)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

cv2.destroyAllWindows()
vs.stop()