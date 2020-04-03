import cv2 as cv
import numpy as np

start = 0
mazeHSVLow = np.array([start,0, 0])
mazeHSVHigh = np.array([start+100,100,255])

marbleHSVLow = np.array([30,60,100])
marbleHSVHigh = np.array([50,150,255])

def filterMazeNoise(img):
    kernel = np.ones((7,7),np.uint8)

    img = cv.morphologyEx(img, cv.MORPH_CLOSE, kernel, iterations=1)
    img = cv.dilate(img,kernel,iterations=3)
    img = cv.erode(img,kernel,iterations=2)

    return img

def filterMarbleNoise(img):
    kernel = np.ones((5,5),np.uint8)

    img = cv.erode(img,kernel, iterations=1)
    img = cv.dilate(img,kernel,iterations=5)
    img = cv.erode(img,kernel, iterations=5)
    img = cv.dilate(img,kernel, iterations=2)

    return img

def segmentImg(frame):

    maze = cv.inRange(frame, mazeHSVLow, mazeHSVHigh)
    maze = filterMazeNoise(maze)

    frameHSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    marbleframe = cv.inRange(frameHSV,marbleHSVLow,marbleHSVHigh)
    marbleframe = cv.subtract(marbleframe,maze)
    marbleframe = filterMarbleNoise(marbleframe)
    marbleframe = cv.Canny(marbleframe,100,255)

    contours, hierarchy = cv.findContours(marbleframe, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    circle = []
    center = (0,0)

    if len(contours):
        for countour in contours:
            c, r = cv.minEnclosingCircle(countour)
            if len(countour) > len(circle):
                circle = countour

        center, radius = cv.minEnclosingCircle(circle)
        center = (int(center[0]),int(center[1]))
        radius = int(radius)


        ####### shows circle tracking marble
        # cv.circle(frame,center,15,(255,0,0),2)
        # cv.imshow('Marble',frame)
        # cv.waitKey(200)

    return maze, center

cap = cv.VideoCapture('maze_transform.avi')

ret = True

while ret:
    
    if cv.waitKey(1) == 27:
        break

    ret, frame = cap.read()


    if ret:
        maze, ballLoc = segmentImg(frame)
