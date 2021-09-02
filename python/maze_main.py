#! /usr/bin/env python3
import cv2 as cv
import numpy as np
from mazeSolver import mazeSolver
from ballControl import ballController
from servoControl import ServoControl
import time


show_stuff = True
solver = mazeSolver(show_stuff)
controller = ballController()
servo = ServoControl(top_center=0.15, bot_center=0.0)
goal = 131 # goal position read from left to right, top to bottom

# vid = cv.VideoCapture('../images/maze_newcam.avi')
vid = cv.VideoCapture(0)

videoFile = "../mazesolver.avi"
vOut = None

solveFlag =False
pause = True

start = time.time()

marble_pos_tm1 = None
while vid.isOpened():
    ret, frame = vid.read()
    if not ret:
        break
    frame = frame[125:350 , 225:450]

    marble_pos, waypoint, policyframe = solver.solveMaze(frame, goal, solveFlag)

    print(marble_pos, waypoint)

    if pause == False and marble_pos is not None and waypoint is not None:

        # filter the position of the marble
        if marble_pos_tm1 is None:
            marble_pos_tm1 = marble_pos
        else:
            marble_pos = .5*marble_pos + .5*marble_pos_tm1
            marble_pos_tm1 = marble_pos

        Ts = time.time() - start
        print("Ts = ", Ts)
        start = time.time()
        u_x, u_y = controller.update(marble_pos, waypoint, Ts)

        # top servo is in camera y direction, bottom is in camera x direction
        servo.writeCommand(u_y, u_x)

        print("u_x = ", u_x)
        print("u_y = ", u_y)

    solveFlag = False

    cv.imshow('frame', frame)

    cmd = cv.waitKey(1)
    if cmd == ord('q'):         # q = quit
        break
    elif cmd == ord('s'):       # s = solve the maze
        print("Solving")
        solveFlag = True
    elif cmd == ord('g'):       # g = set the goal location
        while True:
            try:
                goal = int(input("Enter a goal location between 0 and 143: "))
                if goal >= 0 and goal <= 143:
                    break
                else:
                    print("Invalid goal location")
            except ValueError:
                print("Integer only, please")
    elif cmd == ord(' '):       # spacebar = pause controller
        pause = not pause
    elif cmd == ord('r'):       # r = reset to level
        servo.writeCommand(0.0, 0.0)

    print("")

    if videoFile is not None and vOut is None and policyframe is not None:
        fourcc = cv.VideoWriter_fourcc(*"XVID")
        vOut = cv.VideoWriter(videoFile, fourcc, 30, (policyframe.shape[0], policyframe.shape[1]))

    if videoFile is not None and policyframe is not None:
        vOut.write(policyframe)

controller.plotCmd()

vid.release()
vOut.release()
cv.destroyAllWindows()
