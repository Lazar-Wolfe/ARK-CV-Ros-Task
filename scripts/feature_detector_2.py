#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from std_msgs.msg import Int32MultiArray


def send_corners():
    result = np.array([])
    pub = rospy.Publisher('corner_topic',Int32MultiArray,queue_size = 10)
    rospy.init_node('corner_node',anonymous=True)
    rate=rospy.Rate(10)
    msg=Int32MultiArray()
    while not rospy.is_shutdown():
        connections = pub.get_num_connections()
        rospy.loginfo("Connections : %d",connections)
        if connections>0:
            rospy.loginfo("Data Sent")
            for id in range(10):
                image = cv2.imread(r"/home/chaitanya/catkin_ws/src/cvros/CV-ROS Task/Segmentation/"+str(id)+"_1.png")
                gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
                gray = np.float32(gray)
                # image, maximum corners, quality, distance between corners
                corners = cv2.goodFeaturesToTrack(gray, 4, 0.1, 30)
                corners = np.int0(corners)

                for corner in corners:
                    x, y = corner.ravel()
                    cv2.circle(image, (x, y), 3, 255, -1)
                result = np.concatenate((result, corners.flatten()), axis=None)
            msg.data = result
            pub.publish(msg)
            break
        rate.sleep()

if __name__ == '__main__':
    try:
        send_corners()
    except rospy.ROSInterruptException:
        pass