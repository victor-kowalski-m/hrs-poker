"""
This file's purpose is the generation of template images of ranks and
suites for comparison with a captured card. Thus, it is to be runned once
before the execution of the proper detection.
It akes a card picture and creates a top-down 200x300 flattened image
of it. Isolates the suit and rank and saves the isolated images.
Runs through A - K ranks and then the 4 suits.
"""

# Import necessary packages
import cv2
import detection
import numpy as np
import os
import subprocess

# Path to save the templates
img_path = os.path.dirname(os.path.abspath(__file__)) + '/Card_Imgs/'

# Card parameters
RANK_WIDTH = 70
RANK_HEIGHT = 125

SUIT_WIDTH = 70
SUIT_HEIGHT = 100

CORNER_WIDTH = 38
CORNER_HEIGHT = 84

CARD_THRESH = 30

# Sets up webcam
cap = cv2.VideoCapture(0)
subprocess.check_call("v4l2-ctl -d /dev/video0 -c exposure_auto=1",shell=True) # 1 manual 3 auto
subprocess.check_call("v4l2-ctl -d /dev/video0 -c exposure_absolute=140",shell=True)
subprocess.check_call("v4l2-ctl -d /dev/video0 -c focus_auto=0",shell=True)
subprocess.check_call("v4l2-ctl -d /dev/video0 -c focus_absolute=20",shell=True) # 25cm height
subprocess.check_call("v4l2-ctl -d /dev/video0 -c zoom_absolute=150",shell=True)
subprocess.check_call("v4l2-ctl -d /dev/video0 -c gain=0",shell=True)
subprocess.check_call("v4l2-ctl -d /dev/video0 -c sharpness=255",shell=True)

# Use counter variable to switch from isolating Rank to isolating Suit
i = 1

# Runs for each rank then for each suit
for Name in ['Ace','Two','Three','Four','Five','Six','Seven','Eight',
             'Nine','Ten','Jack','Queen','King','Spades','Diamonds',
             'Clubs','Hearts']:

    filename = Name + '.jpg'

    c = False
    while not c:

        print('Press "p" to take a picture of ' + filename)
        
        while(True):

            ret, frame = cap.read()
            cv2.imshow("1) original",frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("p"):
                image = frame
                break
            if key == ord("q"):
                cv2.destroyAllWindows()
                exit()

        # Pre-process image
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray,(5,5),0)
        retval, thresh = cv2.threshold(blur,100,255,cv2.THRESH_BINARY)
        cv2.imshow("2) 1st thresh (find contours)", thresh)

        # Find contours and sort them by size
        dummy,cnts,hier = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cnts = sorted(cnts, key=cv2.contourArea,reverse=True)

        # Assume largest contour is the card. If there are no contours, print an error
        flag = 0
        image2 = image.copy()

        if len(cnts) == 0:
            print('No contours found!')
            quit()

        card = cnts[0]

        # Approximate the corner points of the card
        peri = cv2.arcLength(card,True)
        approx = cv2.approxPolyDP(card,0.01*peri,True)
        pts = np.float32(approx)

        x,y,w,h = cv2.boundingRect(card)

        # Flatten the card and convert it to 200x300
        warp = detection.flattener(image,pts,w,h)
        cv2.imshow("3) flattened", warp)

        # Grab corner of card image, zoom, and threshold
        corner = warp[0:CORNER_HEIGHT, 0:CORNER_WIDTH]
        cv2.imshow("4) corner", corner)
        corner_zoom = cv2.resize(corner, (0,0), fx=4, fy=4)
        corner_blur = cv2.GaussianBlur(corner_zoom,(5,5),0)
        retval, corner_thresh = cv2.threshold(corner_blur, 80, 255, cv2. THRESH_BINARY_INV)
        cv2.imshow("5) corner thresh", corner_thresh)

        # Isolate suit or rank
        if i <= 13: # Isolate rank
            rank = corner_thresh[0:194, 0:144] # Grabs portion of image that shows rank
            cv2.imshow("6) crop rank", rank)
            dummy, rank_cnts, hier = cv2.findContours(rank, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            rank_cnts = sorted(rank_cnts, key=cv2.contourArea,reverse=True)
            x,y,w,h = cv2.boundingRect(rank_cnts[0])
            rank_roi = rank[y:y+h, x:x+w]
            rank_sized = cv2.resize(rank_roi, (RANK_WIDTH, RANK_HEIGHT), 0, 0)
            final_img = rank_sized
            cv2.imshow("7) final rank",final_img)

        if i > 13: # Isolate suit
            suit = corner_thresh[170:336, 0:144] # Grabs portion of image that shows suit
            cv2.imshow("6) crop suit", suit)
            dummy, suit_cnts, hier = cv2.findContours(suit, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            suit_cnts = sorted(suit_cnts, key=cv2.contourArea,reverse=True)
            x,y,w,h = cv2.boundingRect(suit_cnts[0])
            suit_roi = suit[y:y+h, x:x+w]
            suit_sized = cv2.resize(suit_roi, (SUIT_WIDTH, SUIT_HEIGHT), 0, 0)
            final_img = suit_sized
            cv2.imshow("7) final suit",final_img)

        # Save image
        print('Press "c" to continue.')
        key = cv2.waitKey(0) & 0xFF
        if key == ord('c'):
            cv2.imwrite(img_path+filename,final_img)
            c = True
            i = i + 1
        if key == ord("q"):
            break


cv2.destroyAllWindows()
