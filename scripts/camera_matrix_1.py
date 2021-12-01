#!/usr/bin/env python
import numpy as np
import re
import cv2 as cv
import rospy
from std_msgs.msg import Float64MultiArray


def read_pose():
    result = np.array([])
    f=256/2
    intrinsic_matrix = np.identity(3)
    intrinsic_matrix[0][0]=f
    intrinsic_matrix[1][1]=f
    intrinsic_matrix[0][2]=256/2
    intrinsic_matrix[1][2]=144/2

    pub = rospy.Publisher('camera_matrix_topic', Float64MultiArray, queue_size=10)
    rospy.init_node('camera_matrix_node',anonymous=True)
    rate=rospy.Rate(10)
    msg = Float64MultiArray()
    while not rospy.is_shutdown():
        connections = pub.get_num_connections()
        rospy.loginfo("Connections : %d",connections)
        if connections>0:
            rospy.loginfo("Data is being sent")
            for id in range(10):
                file = open(r"/home/chaitanya/catkin_ws/src/cvros/CV-ROS Task/Pose/" + str(id) + "_2.txt", "rb")
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
                camera_matrix = np.dot(intrinsic_matrix,extrinsic_matrix)
                result=np.concatenate((result,camera_matrix.flatten()),axis=None)
            msg.data = result
            pub.publish(msg)
            break
        rate.sleep()


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

if __name__ == '__main__':
    try:
        read_pose()
    except rospy.ROSInterruptException:
        pass
