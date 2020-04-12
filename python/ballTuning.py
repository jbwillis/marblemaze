#! /usr/bin/env python3
import cv2 as cv
import numpy as np
from mazeSolver import mazeSolver
from ballControl import ballController
from servoControl import ServoControl
import time


def main():
    show_stuff = True
    solver = mazeSolver(show_stuff)
    controller = ballController()
    servo = ServoControl(top_center=0., bot_center=0.0)

    vid = cv.VideoCapture(0)
    # cv.namedWindow("Empty")

    pause = True

    start = time.time()

    marble_pos_tm1 = None
    while vid.isOpened():
        ret, frame = vid.read()
        frame = frame[125:350 , 225:450]
        cv.imshow("Frame", frame)
        if not ret:
            break

        marble_pos, _ = solver.solveMaze(frame, 0, False)

        waypoint = (200.0, 200.0)

        print(marble_pos, waypoint)

        if pause == False:
            if marble_pos is not None and waypoint is not None:

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
            # else:
                # u_x = 0.0
                # u_y = 0.0


                # top servo is in camera y direction, bottom is in camera x direction
                servo.writeCommand(u_y, u_x)

                print("u_x = ", u_x)
                print("u_y = ", u_y)

        cmd = cv.waitKey(1)
        if cmd == ord('q'):         # q = quit
            break
        elif cmd == ord(' '):       # spacebar = pause controller
            pause = not pause
        elif cmd == ord('r'):       # r = reset to level
            servo.writeCommand(0.0, 0.0)

        print("")


    vid.release()
    cv.destroyAllWindows()

main()
