import cv2 as cv
import numpy as np
from mazeSolver import mazeSolver
from bfs_example import plot_board, plot_solution
import time


show_stuff = True
solver = mazeSolver(show_stuff)
goal = 97 # goal position read from left to right, top to bottom

vid = cv.VideoCapture('../images/maze_blueball.avi')
# vid = cv.VideoCapture(0)

solveFlag =True
pause = True

while vid.isOpened():
    start = time.time()
    ret, frame = vid.read()
    if not ret:
        break

    marble_pos, waypoint = solver.solveMaze(frame, goal, solveFlag)
    print(marble_pos, waypoint)
    print(1/(time.time() - start))
    # solveFlag = False

    cv.imshow('frame', frame)
    cv.waitKey(0)

    # cmd = cv.waitKey(1)
    # if cmd == ord('q'):         # q = quit
    #     break
    # elif cmd == ord('s'):       # s = solve the maze
    #     solveFlag = True
    # elif cmd == ord('g'):       # g = set the goal location
    #     while True:
    #         try:
    #             goal = int(input("Enter a goal location between 0 and 143: "))
    #             if goal >= 0 and goal <= 143:
    #                 break
    #             else:
    #                 print("Invalid goal location")
    #         except ValueError:
    #             print("Integer only, please")
    # elif cmd == ord(' '):       # spacebar = pause controller
    #     pause = not pause



vid.release()
cv.destroyAllWindows()
