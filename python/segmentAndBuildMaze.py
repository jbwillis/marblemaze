import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import time

start = 0
mazeHSVLow = np.array([start, 0, 0])
mazeHSVHigh = np.array([start + 100, 100, 255])

marbleHSVLow = np.array([30, 60, 100])
marbleHSVHigh = np.array([50, 150, 255])


def filterMazeNoise(img):
    kernel = np.ones((7, 7), np.uint8)

    img = cv.morphologyEx(img, cv.MORPH_CLOSE, kernel, iterations=1)
    img = cv.dilate(img, kernel, iterations=3)
    img = cv.erode(img, kernel, iterations=2)

    return img


def filterMarbleNoise(img):
    kernel = np.ones((5, 5), np.uint8)

    img = cv.erode(img, kernel, iterations=1)
    img = cv.dilate(img, kernel, iterations=5)
    img = cv.erode(img, kernel, iterations=5)
    img = cv.dilate(img, kernel, iterations=2)

    return img


def segmentImg(frame):
    maze = cv.inRange(frame, mazeHSVLow, mazeHSVHigh)
    maze = filterMazeNoise(maze)

    frameHSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    marbleframe = cv.inRange(frameHSV, marbleHSVLow, marbleHSVHigh)
    marbleframe = cv.subtract(marbleframe, maze)
    marbleframe = filterMarbleNoise(marbleframe)
    marbleframe = cv.Canny(marbleframe, 100, 255)

    contours, hierarchy = cv.findContours(marbleframe, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    circle = []
    center = (0, 0)

    if len(contours):
        for countour in contours:
            c, r = cv.minEnclosingCircle(countour)
            if len(countour) > len(circle):
                circle = countour

        center, radius = cv.minEnclosingCircle(circle)
        center = (int(center[0]), int(center[1]))
        radius = int(radius)

        ####### shows circle tracking marble
        # cv.circle(frame,center,15,(255,0,0),2)
        # cv.imshow('Marble',frame)
        # cv.waitKey(200)

    return maze, center

def drawMazeDims(maze):
    mazeLen = 12
    mazeWallThick = 17
    firstWallThick = 7
    mazeGapThick = 16

    mazeColor = cv.cvtColor(maze, cv.COLOR_GRAY2RGB)
    mazeX = 0
    mazeY = 0
    imgHeight = maze.shape[1]
    imgWidth = maze.shape[0]
    for j in range(mazeLen):
        cv.line(mazeColor, (mazeX, 0), (mazeX, imgHeight), (0, 0, 255), 2)
        cv.line(mazeColor, (0, mazeY), (imgWidth, mazeY), (0, 0, 255), 2)
        if j == 0:
            mazeX = mazeX + firstWallThick
            mazeY = mazeY + firstWallThick
        else:
            mazeX = mazeX + mazeWallThick
            mazeY = mazeY + mazeWallThick
        cv.line(mazeColor, (mazeX, 0), (mazeX, imgHeight), (0, 255, 0), 2)
        cv.line(mazeColor, (0, mazeY), (imgWidth, mazeY), (0, 255, 0), 2)
        mazeX = mazeX + mazeGapThick
        mazeY = mazeY + mazeGapThick

    cv.imshow('segmented maze', mazeColor)

def buildMaze(maze):
    mazeLen = 12
    mazeWallThick = 17
    firstWallThick = 7
    mazeGapThick = 16
    mazeGapAndWallThick = mazeWallThick + mazeGapThick
    checkSize = (mazeWallThick * mazeGapThick) * 0.3
    nodalMaze = np.zeros((mazeLen * mazeLen, 4))

    mazeX = firstWallThick
    for k in range(mazeLen):
        mazeY = firstWallThick
        for j in range(mazeLen):
            if k == 0:
                up = 0;
            else:
                up = int(np.count_nonzero(maze[mazeX-mazeWallThick:mazeX,mazeY:mazeY+mazeGapThick]) < checkSize)
            if k == mazeLen - 1:
                down = 0
            else:
                down = int(np.count_nonzero(maze[mazeX + mazeGapThick:mazeX + mazeGapAndWallThick, mazeY:mazeY + mazeGapThick]) < checkSize)
            if j == 0:
                left = 0
            else:
                left = int(np.count_nonzero(maze[mazeX:mazeX+mazeGapThick,mazeY-mazeWallThick:mazeY]) < checkSize)
            if j == mazeLen - 1:
                right = 0
            else:
                right = int(np.count_nonzero(maze[mazeX:mazeX+mazeGapThick,mazeY+mazeGapThick:mazeY+mazeGapAndWallThick]) < checkSize)
            nodalMaze[k*mazeLen + j, :] = np.array([up,down,left,right]).reshape(1,4)
            mazeY = mazeY + mazeGapAndWallThick
        mazeX = mazeX + mazeGapAndWallThick

    return nodalMaze

def plot_board(size, edges, goal):
    plt.figure(1)
    w = size[0]
    h = size[1]

    x_row = [i for i in range(w)]
    x = []
    y = []
    for i in range(h):
        x += x_row
        y += [-i for j in range(w)]
    plt.scatter(x,y)

    for i in range(w*h):
        xval = i%w
        yval = -np.floor(i/w)
        for j in range(4):
            if edges[i,j]:
                if j == 0: #up
                    plt.plot([xval,xval],[yval,yval+1])
                elif j == 1: #down
                    plt.plot([xval,xval],[yval,yval-1])
                elif j == 2: #left
                    plt.plot([xval,xval-1],[yval,yval])
                elif j == 3: #right
                    plt.plot([xval,xval+1],[yval,yval])

    #plt.plot(goal%w, -np.floor(goal/w), 'r*')



if __name__=="__main__":
    cap = cv.VideoCapture('../images/maze_transform.avi')

    ret = True
    mazeBuiltFlag = False

    while ret:

        if cv.waitKey(1) == ord("q"):
            break

        ret, frame = cap.read()

        maze, ballLoc = segmentImg(frame)
        if mazeBuiltFlag is not True:
            #mazeBuiltFlag = True
            nodalMaze = buildMaze(maze)
            plot_board([12,12],nodalMaze,6)
            plt.show(block=False)
            plt.pause(0.01)
            #time.sleep(0.1)
            print("hey")
            #drawMazeDims(maze)