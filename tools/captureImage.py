#! /usr/bin/env python3

import cv2
import numpy as np
import argparse

parser = argparse.ArgumentParser(description='Capture an image from the given camera number')


parser.add_argument('cam', help='OpenCV camera number (0 indexed)', type=int)

args = parser.parse_args()

print("Press 'c' to capture an image or 'q' to quit")

cap = cv2.VideoCapture(args.cam)

while True:
    ret, frame = cap.read()

    cv2.imshow('Frame', frame)

    kp = cv2.waitKey(1)
    if (kp & 0xFF) == ord('c'):
        fn = input("Enter the filename: ")
        cv2.imwrite(fn, frame)

    elif (kp & 0xFF) == ord('q'):
        break
        
cap.release()
cv2.destroyAllWindows()

