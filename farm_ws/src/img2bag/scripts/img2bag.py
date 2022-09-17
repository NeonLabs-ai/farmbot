#!/usr/bin/env python3
import rospy  # Python library for ROS
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import os


def publish_message():

    pub = rospy.Publisher('/bottom_camera/image_raw', Image, queue_size=10)
    rospy.init_node('video_pub_py', anonymous=True)

    rate = rospy.Rate(10)  # 10hz

    path = '/home/bhavik/neonLabs-ai/personal/farmbot/deep_learning/dataset/test/images/'
    imgs = os.listdir(path)
    imgs.sort()

    l = len(imgs)
    i = 0

    br = CvBridge()

    while not rospy.is_shutdown():

        if i == l -1:
            i = 0

        frame = cv2.imread(path + imgs[i])

        pub.publish(br.cv2_to_imgmsg(frame))
        rospy.loginfo('publishing video frame')
        i +=1 

        rate.sleep()


if __name__ == '__main__':
    try:
        publish_message()
    except rospy.ROSInterruptException:
        pass
