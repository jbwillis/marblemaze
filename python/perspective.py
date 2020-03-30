import numpy as np
import cv2 as cv
import time

def get_max_contour(binary):
    contours,hierarchy = cv.findContours(binary, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    areas = [cv.contourArea(c) for c in contours]
    max_index = np.argmax(areas)
    return contours[max_index]


def get_corners_color(frame):
    dots_lower_hsv = (150,100,80)
    dots_upper_hsv = (185,185,180)

    dots_lower_bgr = (25,20,90)
    dots_upper_bgr = (90,85,255)

    frame = cv.medianBlur(frame, 5)
    frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    # frame_dots_hsv = cv.inRange(frame_hsv, dots_lower_hsv, dots_upper_hsv)
    frame_dots = cv.inRange(frame, dots_lower_bgr, dots_upper_bgr)
    # frame_dots = np.bitwise_and(frame_dots_hsv, frame_dots_rgb)
    erosion_size = 2
    element = cv.getStructuringElement(cv.MORPH_ELLIPSE, (2*erosion_size + 1, 2*erosion_size+1), (erosion_size, erosion_size))
    frame_dots = cv.erode(frame_dots, element)
    frame_dots = cv.dilate(frame_dots, element)

    cv.imshow('rgb',frame[:,:,1])
    cv.imshow('sat',frame_hsv[:,:,1])

    return frame_dots

def get_corners(frame):
    frame = cv.medianBlur(frame, 5)
    ret,walls_b = cv.threshold(frame[:,:,0],90,255,cv.THRESH_BINARY_INV)
    ret,walls_g = cv.threshold(frame[:,:,1],80,255,cv.THRESH_BINARY_INV)
    ret,walls_r = cv.threshold(frame[:,:,2],80,255,cv.THRESH_BINARY_INV)
    walls = cv.bitwise_or(cv.bitwise_or(walls_g, walls_b), walls_r)

    erosion_size = 1
    element = cv.getStructuringElement(cv.MORPH_ELLIPSE, (2*erosion_size + 1, 2*erosion_size+1), (erosion_size, erosion_size))
    walls = cv.erode(walls, element)
    walls = cv.dilate(walls, element)

    erosion_size = 2
    element = cv.getStructuringElement(cv.MORPH_ELLIPSE, (2*erosion_size + 1, 2*erosion_size+1), (erosion_size, erosion_size))
    walls = cv.dilate(walls, element)
    walls = cv.erode(walls, element)

    # walls_flood = walls.copy()
    # h,w = walls.shape[:2]
    # mask = np.zeros((h+2,w+2), np.uint8)
    # cv.floodFill(walls_flood, mask, (0,0), 255)
    # walls_flood_inv = cv.bitwise_not(walls_flood)
    # board = walls | walls_flood_inv
    #
    # erosion_size = 10
    # element = cv.getStructuringElement(cv.MORPH_ELLIPSE, (2*erosion_size + 1, 2*erosion_size+1), (erosion_size, erosion_size))
    # board = cv.erode(board, element)
    # board = cv.dilate(board, element)

    wall_cnt = get_max_contour(walls)

    epsilon = 0.1*cv.arcLength(wall_cnt, True)
    approx = cv.approxPolyDP(wall_cnt, epsilon, True).reshape(4,2)
    approx = approx[np.argsort(approx[:, 0])]
    left_y = np.argmin([approx[0,1],approx[1,1]])
    right_y = np.argmin([approx[2,1],approx[3,1]])
    if left_y:
        approx[[0, 1]] = approx[[1, 0]]
    if right_y:
        approx[[2, 3]] = approx[[3, 2]]

    return walls, approx


vid = cv.VideoCapture('../images/maze.avi')
fourcc = cv.VideoWriter_fourcc(*'XVID')
vid_out = cv.VideoWriter('../images/maze_transform.avi',fourcc, 30.0, (400,400))

dst_corners = np.float32([[0,0],[0,400],[400,0],[400,400]])

while vid.isOpened():
    ret, frame = vid.read()
    if not ret:
        break

    start = time.time()
    try:
        board, corners = get_corners(frame)
    except:
        print("Bad frame! Skipping...")
        continue
    # frame_corners = frame.copy()
    # for i in range(corners.shape[0]):
    #     cv.circle(frame_corners, (int(corners.item(i,0)), int(corners.item(i,1))), 5, (0,255,0))

    H,_ = cv.findHomography(corners, dst_corners)
    warped = cv.warpPerspective(frame, H, (400,400))
    print(time.time()-start)

    vid_out.write(warped)
    # cv.imshow('vid',frame_corners)
    cv.imshow('board_cnt', board)
    cv.imshow('warped', warped)
    # cv.waitKey(0)
    if cv.waitKey(1) == ord('q'):
        break

cv.destroyAllWindows()
vid.release()
vid_out.release()
