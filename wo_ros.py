import math

import numpy as np
import re
import cv2 as cv
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import csv

# 10 different matrices of size 3x4
camera_matrices = np.zeros((10,3,4))

# 10 different matrices of size 2xN where N is the number of corners we want
corner_matrices = np.zeros((10,2,4))

safe= np.zeros((4,3))

def read_pose():
    global camera_matrices
    f=256/2
    intrinsic_matrix = np.identity(3)
    intrinsic_matrix[0][0]=f
    intrinsic_matrix[1][1]=f
    intrinsic_matrix[0][2]=256/2
    intrinsic_matrix[1][2]=144/2
    for id in range(10):
        file = open(r"C:/Users/c28ga/Documents/ARK/CV-ROS Task/Pose/" + str(id) + "_2.txt", "rb")
        quaternion = []
        position = []
        data = str(file.read())
        # print(data)
        quaternion = (re.findall(r"[-+]?\d*\.\d+|\d+", data))[:4]
        position = (re.findall(r"[-+]?\d*\.\d+|\d+", data))[5:]
        # print(quaternion)
        # print(position)
        file.close()
        extrinsic_matrix = extrinsic(quaternion, position)
        camera_matrix = np.dot(intrinsic_matrix, extrinsic_matrix)
        # print(camera_matrix)
        camera_matrices[id]=camera_matrix
    # print(camera_matrices)
def extrinsic(q, p):
    q0 = float(q[0])
    q1 = float(q[1])
    q2 = float(q[2])
    q3 = float(q[3])
    p0 = float(p[0])
    p1 = float(p[1])
    p2 = float(p[2])

    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
    translation_matrix = np.array([[p0],
                                   [p1],
                                   [p2]])
    # print(rot_matrix)
    # print(translation_matrix)
    extrinsic_matrix = np.array([[r00, r01, r02, p0],
                                 [r10, r11, r12, p1],
                                 [r20, r21, r22, p2]])
    # print(extrinsic_matrix)
    return extrinsic_matrix

def get_points():
    global corner_matrices
    for id in range(10):
        image = cv.imread(r"C:/Users/c28ga/Documents/ARK/CV-ROS Task/Segmentation/" +str(id) + "_1.png")
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        gray = np.float32(gray)
        # image, maximum corners, quality, distance between corners
        corners = cv.goodFeaturesToTrack(gray, 4, 0.1, 30)
        corners = np.int0(corners)

        for corner in corners:
            x, y = corner.ravel()
            cv.circle(image, (x, y), 3, 255, -1)
        temp = corners.flatten()
        temp_reshape = temp.reshape(4,2)
        # print(temp_reshape.transpose())
        corner_matrices[id] = temp_reshape.transpose()
    # print(corner_matrices)


def triangulate():
    global safe
    for i in range(9):
    # i=0
        world_points=cv.triangulatePoints(camera_matrices[i],camera_matrices[i+1],corner_matrices[i],corner_matrices[i+1])
        # world_points[:,0]=world_points[:,0]/world_points[3][0]
        # world_points[:,1]=world_points[:,0]/world_points[3][1]
        # world_points[:,2]=world_points[:,0]/world_points[3][2]
        # world_points[:,3]=world_points[:,0]/world_points[3][3]

        temp = world_points[:-1].transpose()

        # print(temp)
        # print(world_points)
        # print(world_points[:-1])

        safe = np.concatenate((safe,temp))


    safe = np.delete(safe, (0),axis = 0)
    safe = np.delete(safe, (0),axis = 0)
    safe = np.delete(safe, (0),axis = 0)
    safe = np.delete(safe, (0),axis = 0)
    xs = safe[:,0]
    ys = safe[:,1]
    zs = safe[:,2]
    print(safe)

read_pose()
get_points()
triangulate()

xs = safe[:,0]
ys = safe[:,1]
zs = safe[:,2]
ax = plt.axes(projection ="3d")
ax.scatter3D(xs, ys, zs, color="green")
plt.title("simple 3D scatter plot")

# show plot
plt.show()

dist = []

for i in range(36):
    for j in range(i+1,36,1):
        x = (safe[i][0]-safe[j][0])**2
        y = (safe[i][1]-safe[j][1])**2
        z = (safe[i][2]-safe[j][2])**2
        dist.append(math.sqrt(x+y+z))
print(dist)

