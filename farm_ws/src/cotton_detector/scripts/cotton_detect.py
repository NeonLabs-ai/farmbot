import rospy  
from sensor_msgs.msg import Image  
from cv_bridge import CvBridge 
import cv2 
import time
from geometry_msgs.msg import Point
import numpy as np

pub = rospy.Publisher('/cottons', Point)

def detectCottonBall(image):

    image_height, image_width, _ = image.shape
    rospy.loginfo(f"{image.shape}")

    # frame = cv2.resize(image,(512,512))
    
    l_white = np.array([0,0,168])
    u_white = np.array([172,111,255])
    
    output_image = image.copy()

    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,l_white,u_white)

    contours,_= cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    max_contour = contours[0]
    for contour in contours:
        if cv2.contourArea(contour)>cv2.contourArea(max_contour):
            max_contour=contour

    contour=max_contour
    approx=cv2.approxPolyDP(contour, 0.01*cv2.arcLength(contour,True),True)
    x,y,w,h=cv2.boundingRect(approx)
    cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),4)

    cv2.imshow("frame",image)
    cv2.imshow("mask",mask)


    data = Point()
    data.x = x
    data.y = y
    data.z = max(w/2, h/2)
    

    pub.publish(data)


    cv2.waitKey(1)


def callback(data):

    br = CvBridge()

    rospy.loginfo("receiving video frame")

    current_frame = br.imgmsg_to_cv2(data)

    detectCottonBall(current_frame)

    # cv2.imshow("camera", current_frame)

    cv2.waitKey(1)


def receive_message():

    rospy.init_node('cotton_detector', anonymous=True)

    rospy.Subscriber('/bottom_camera/image_raw', Image, callback)
    

    rospy.spin()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    receive_message()
