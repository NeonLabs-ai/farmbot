#!/usr/bin/env python3
import rospy  
from sensor_msgs.msg import Image  
from cv_bridge import CvBridge  
import cv2  
import os


def publish_message():

    pub = rospy.Publisher('/bottom_camera/image_raw', Image, queue_size=10)
    rospy.init_node('video_pub_py', anonymous=True)

    rate = rospy.Rate(10)  # 10hz
    cap = cv2.VideoCapture(0)
    br = CvBridge()

    while not rospy.is_shutdown():

        ret, frame = cap.read()

        if ret == True:
            rospy.loginfo('publishing video frame')
            pub.publish(br.cv2_to_imgmsg(frame))
             
    rate.sleep()



if __name__ == '__main__':
    try:
        publish_message()
    except rospy.ROSInterruptException:
        pass
