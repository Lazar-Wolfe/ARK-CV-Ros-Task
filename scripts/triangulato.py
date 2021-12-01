#!/usr/bin/env python
import cv2
import numpy as np

for id in range(10):
    image = cv2.imread(r"/home/chaitanya/catkin_ws/src/cvros/CV-ROS Task/Segmentation/" + str(id) + "_1.png")
    color = cv2.imread(r"/home/chaitanya/catkin_ws/src/cvros/CV-ROS Task/RGB/" + str(id) + "_0.png")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = np.float32(gray)
    # image, maximum corners, quality, distance between corners
    corners = cv2.goodFeaturesToTrack(gray, 4, 0.1, 30)
    corners = np.int0(corners)

    for corner in corners:
        x, y = corner.ravel()
        cv2.circle(color, (x, y), 3, 255, -1)
    print(corners)
    cv2.imshow(str(id),color)
    cv2.waitKey(0)