import numpy as np
import cv2 as cv
import time

def get_max_contour(binary):
    contours, hierarchy = cv.findContours(binary, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    areas = [cv.contourArea(c) for c in contours]
    max_index = np.argmax(areas)
    return contours[max_index]


def get_corners_color(frame):
    dots_lower_hsv = (10,0,10)
    dots_upper_hsv = (100,255,80)

    # frame = cv.medianBlur(frame, 5)
    frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    ret,sat = cv.threshold(frame_hsv[:,:,1],150,255,cv.THRESH_BINARY)
    erosion_size = 2
    element = cv.getStructuringElement(cv.MORPH_ELLIPSE, (2*erosion_size + 1, 2*erosion_size+1), (erosion_size, erosion_size))
    sat = cv.erode(sat, element)
    sat = cv.dilate(sat, element)
    masked = cv.bitwise_and(frame_hsv, frame_hsv, mask=sat)
    dots = cv.inRange(masked, dots_lower_hsv, dots_upper_hsv)
    # frame_dots_hsv = cv.inRange(frame_hsv, dots_lower_hsv, dots_upper_hsv)
    # frame_dots = cv.inRange(frame, dots_lower_bgr, dots_upper_bgr)
    # frame_dots = np.bitwise_and(frame_dots_hsv, frame_dots_rgb)
    # erosion_size = 2
    # element = cv.getStructuringElement(cv.MORPH_ELLIPSE, (2*erosion_size + 1, 2*erosion_size+1), (erosion_size, erosion_size))
    # frame_dots = cv.erode(frame_dots, element)
    # frame_dots = cv.dilate(frame_dots, element)

    cv.imshow('hsv0',frame_hsv[:,:,0])
    cv.imshow('hsv1',frame_hsv[:,:,1])
    cv.imshow('hsv2',frame_hsv[:,:,2])
    cv.imshow('sat',dots)
    cv.imshow('frame',frame)

    return None

def get_corners_cvx_hull(frame):
    frame = cv.medianBlur(frame[:,:590,:], 5)
    ret,walls_b = cv.threshold(frame[:,:,0],90,255,cv.THRESH_BINARY_INV)
    ret,walls_g = cv.threshold(frame[:,:,1],80,255,cv.THRESH_BINARY_INV)
    ret,walls_r = cv.threshold(frame[:,:,2],30,255,cv.THRESH_BINARY_INV)
    walls = cv.bitwise_or(cv.bitwise_or(walls_g, walls_b), walls_r)
    cv.imshow('rgb1', frame[:,:,0])
    cv.imshow('rgb2', frame[:,:,1])
    cv.imshow('rgb3', frame[:,:,2])
    cv.imshow('walls', walls_r)

    # erosion_size = 1
    # element = cv.getStructuringElement(cv.MORPH_ELLIPSE, (2*erosion_size + 1, 2*erosion_size+1), (erosion_size, erosion_size))
    # walls = cv.erode(walls, element)
    # walls = cv.dilate(walls, element)
    #
    # erosion_size = 2
    # element = cv.getStructuringElement(cv.MORPH_ELLIPSE, (2*erosion_size + 1, 2*erosion_size+1), (erosion_size, erosion_size))
    # walls = cv.dilate(walls, element)
    # walls = cv.erode(walls, element)

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

    # wall_cnt = get_max_contour(walls)
    wall_cnt, _ = cv.findContours(walls_r, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    contour = np.vstack([wall_cnt[i] for i in range(len(wall_cnt))])
    hull = cv.convexHull(contour)
    hull_img = np.zeros(walls_r.shape)
    cv.drawContours(hull_img,[hull],-1,255,2);
    cv.imshow('hull',hull_img)

    epsilon = 0.1*cv.arcLength(hull, True)
    approx = cv.approxPolyDP(hull, epsilon, True).reshape(4,2)
    approx = approx[np.argsort(approx[:, 0])]
    left_y = np.argmin([approx[0,1],approx[1,1]])
    right_y = np.argmin([approx[2,1],approx[3,1]])
    if left_y:
        approx[[0, 1]] = approx[[1, 0]]
    if right_y:
        approx[[2, 3]] = approx[[3, 2]]

    return walls, approx

def get_corners(frame):
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    ret,walls = cv.threshold(gray,180,255,cv.THRESH_BINARY_INV)

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



    cv.imshow('gray', gray)
    cv.imshow('walls',walls)
    return walls, approx

def refine_corners(frame, corners):
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    qualityLevel = 0.01
    minDistance = 10
    blockSize = 3
    gradientSize = 3
    useHarrisDetector = False
    k = 0.04

    criteria = (cv.TERM_CRITERIA_EPS + cv.TermCriteria_COUNT, 40, 0.001)

    better_corners = np.zeros((4,1,2))
    for i in range(4):
        x = corners.item(i,0)
        y = corners.item(i,1)
        small = gray[y-10:y+10, x-10:x+10]
        corner = cv.goodFeaturesToTrack(small, 1, qualityLevel, minDistance, None, \
            blockSize=blockSize, gradientSize=gradientSize, useHarrisDetector=useHarrisDetector, k=k)
        # corner = cv.cornerSubPix(small, corner, (5,5), (-1,-1), criteria)

        better_corners[i,0,:] = corner[0,0,:] + np.array([x-10, y-10])

    return corners



vid = cv.VideoCapture('../images/maze_newcam.avi')
# fourcc = cv.VideoWriter_fourcc(*'XVID')
# vid_out = cv.VideoWriter('../images/maze_transform.avi',fourcc, 30.0, (400,400))

dst_corners = np.float32([[0,0],[0,400],[400,0],[400,400]])
i = 0
while vid.isOpened():
    ret, frame = vid.read()
    if not ret:
        break

    if i < 60:
        i += 1
        continue
    try:
        # board, corners = get_corners(frame)
        corners = get_corners_color(frame)

    except:
        print("Bad frame! Skipping...")
        continue
    # corners = refine_corners(frame, corners)
    print(corners)
    # frame_corners = frame.copy()
    # for i in range(corners.shape[0]):
    #     cv.circle(frame_corners, (int(corners.item(i,0)), int(corners.item(i,1))), 5, (0,255,0))

    # H,_ = cv.findHomography(corners, dst_corners)
    # warped = cv.warpPerspective(frame, H, (400,400))
    # print(time.time()-start)

    # vid_out.write(warped)
    # cv.imshow('vid',frame_corners)
    # cv.imshow('board_cnt', board)
    # cv.imshow('warped', warped)
    cv.waitKey(0)
    # if cv.waitKey(1) == ord('q'):
    #     break

cv.destroyAllWindows()
# vid.release()
# vid_out.release()
