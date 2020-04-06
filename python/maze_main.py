import cv2 as cv
import numpy as np
from mazeSolver import mazeSolver
from bfs_example import plot_board, plot_solution
import time


solve_every_time = True
show_stuff = True
solver = mazeSolver(solve_every_time, show_stuff)
goal = 97 # goal position read from left to right, top to bottom

vid = cv.VideoCapture('../images/maze.avi')
#vid = cv.VideoCapture(0)

while vid.isOpened():
    start = time.time()
    ret, frame = vid.read()
    if not ret:
        break

    marble_pos, waypoint = solver.solveMaze(frame, goal)
    print(marble_pos, waypoint)
    print(1/(time.time() - start))

    cv.imshow('frame', frame)
    # cv.waitKey(0)
    if cv.waitKey(1) == ord('q'):
        break

vid.release()
cv.destroyAllWindows()
