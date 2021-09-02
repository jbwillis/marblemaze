#! /usr/bin/env python3

import cv2
import numpy as np
import argparse

parser = argparse.ArgumentParser(description='Capture an image from the given camera number')


parser.add_argument('cam', help='OpenCV camera number (0 indexed)', type=int)

parser.add_argument('--video', help='Capture and save video', type=str)

args = parser.parse_args()

print("Press 'c' to capture an image or 'q' to quit")

cap = cv2.VideoCapture(args.cam)


vOut = None

while True:
    ret, frame = cap.read()

    frame = frame[125:350 , 225:450]
    cv2.imshow('Frame', frame)

    kp = cv2.waitKey(1)
    if (kp & 0xFF) == ord('c'):
        fn = input("Enter the filename: ")
        cv2.imwrite(fn, frame)

    elif (kp & 0xFF) == ord('q'):
        break

    if args.video is not None and vOut is None:
        fourcc = cv2.VideoWriter_fourcc(*"XVID")
        vOut = cv2.VideoWriter(args.video, fourcc, 30, (frame.shape[0], frame.shape[1]))

    if args.video is not None:
        vOut.write(frame)
        
# vOut.release()
cap.release()
cv2.destroyAllWindows()

