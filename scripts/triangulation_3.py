#!/usr/bin/env python
import numpy as np
import cv2
import math
import rospy
from std_msgs.msg import Float64MultiArray,Int32MultiArray

camera_matrix=np.array([])
corner_matrix=np.array([])
def callback(data):
    global camera_matrix
    temp=np.asarray(data.data)
    camera_matrix = temp.reshape(10,3,4)
    # print(type(camera_matrix[0]))
    # rospy.loginfo(camera_matrix)

def callback1(data):
    global corner_matrix
    temp=np.asarray(data.data)
    corner_matrix = temp.reshape(10,2,4)
    # print(corner_matrix[0].shape)
    # print(type(corner_matrix[0].shape))
    # rospy.loginfo(corner_matrix)
    triangulation = cv2.triangulatePoints(camera_matrix[0],camera_matrix[1],corner_matrix[0],corner_matrix[1])
    print(triangulation)
    print(np.delete(triangulation,(0),axis=0))
    distance(np.delete(triangulation,(0),axis=0))


def listener():
    rospy.init_node("Subscriber_Node",anonymous=True)
    rospy.Subscriber('camera_matrix_topic',Float64MultiArray,callback)
    rospy.Subscriber('corner_topic',Int32MultiArray,callback1)
    rospy.spin()

def distance(world_points):
    print("Distances")
    for i in range(3):
        x = (abs(world_points[0][i])-abs(world_points[0][i+1]))**2
        y = (abs(world_points[1][i])-abs(world_points[1][i+1]))**2
        z = (abs(world_points[2][i])-abs(world_points[2][i+1]))**2
        print(math.sqrt(x+y+z))



if __name__ == '__main__':
    listener()
