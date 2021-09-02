#! /usr/bin/env python3

import cv2
import numpy as np
import argparse
import time
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(description='Capture an image from the given camera number')


parser.add_argument('cam', help='OpenCV camera number (0 indexed)', type=int)

args = parser.parse_args()

# read once to get the camera started
cap = cv2.VideoCapture(args.cam)
ret, frame = cap.read()

t_vec = np.zeros(100)

for i in range(100):
    t0 = time.time()
    ret, frame = cap.read()
    t1 = time.time()

    t_vec[i] = t1-t0


print("Average FPS: ", 1./np.mean(t_vec))
print("Max FPS: ", 1./np.min(t_vec))
print("Min FPS: ", 1./np.max(t_vec))
        
plt.hist(1./t_vec)
plt.show()
cap.release()
cv2.destroyAllWindows()

