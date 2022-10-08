import rospy  
from sensor_msgs.msg import Image  
from cv_bridge import CvBridge 
import cv2 
import time
from geometry_msgs.msg import Point

pub = rospy.Publisher('/detections', Point)

opencv_dnn_model = cv2.dnn.readNetFromCaffe(prototxt="/home/bk/models/deploy.prototxt",
                                            caffeModel="/home/bk/models/res10_300x300_ssd_iter_140000_fp16.caffemodel")

def cvDnnDetectFaces(image, opencv_dnn_model, min_confidence=0.5, display = True):

    image_height, image_width, _ = image.shape
    rospy.loginfo(f"{image.shape}")

    output_image = image.copy()

    preprocessed_image = cv2.dnn.blobFromImage(image, scalefactor=1.0, size=(300, 300),
                                               mean=(104.0, 117.0, 123.0), swapRB=False, crop=False)

    opencv_dnn_model.setInput(preprocessed_image)
    
    results = opencv_dnn_model.forward()    

    for face in results[0][0]:
        
        face_confidence = face[2]
        
        if face_confidence > min_confidence:

            bbox = face[3:]

            x1 = int(bbox[0] * image_width)
            y1 = int(bbox[1] * image_height)
            x2 = int(bbox[2] * image_width)
            y2 = int(bbox[3] * image_height)

            data = Point()
            data.x = (x1 + x2)/2
            data.y = (y1 + y2)/2
            data.z = (y2 - y1)/2
            pub.publish(data)

            cv2.rectangle(output_image, pt1=(x1, y1), pt2=(x2, y2), color=(0, 255, 0), thickness=image_width//200)

            cv2.rectangle(output_image, pt1=(x1, y1-image_width//20), pt2=(x1+image_width//16, y1),
                          color=(0, 255, 0), thickness=-1)

            cv2.putText(output_image, text=str(round(face_confidence, 1)), org=(x1, y1-25), 
                        fontFace=cv2.FONT_HERSHEY_COMPLEX, fontScale=image_width//700,
                        color=(255,255,255), thickness=image_width//200)

    if display:

        # cv2.putText(output_image, text='Time taken: '+str(round(end - start, 2))+' Seconds.', org=(10, 65),
        #             fontFace=cv2.FONT_HERSHEY_COMPLEX, fontScale=image_width//700,
        #             color=(0,0,255), thickness=image_width//500)
        
        # plt.figure(figsize=[20,20])
        # plt.subplot(121);plt.imshow(image[:,:,::-1]);plt.title("Original Image");plt.axis('off');
        # plt.subplot(122);plt.imshow(output_image[:,:,::-1]);plt.title("Output");plt.axis('off');
        
        cv2.imshow("camera", output_image)
        cv2.waitKey(1)
    else:
        
        return output_image, results

def callback(data):

    br = CvBridge()

    rospy.loginfo("receiving video frame")

    current_frame = br.imgmsg_to_cv2(data)

    cvDnnDetectFaces(current_frame, opencv_dnn_model)

    # cv2.imshow("camera", current_frame)

    cv2.waitKey(1)


def receive_message():

    rospy.init_node('video_sub_py', anonymous=True)

    rospy.Subscriber('/bottom_camera/image_raw', Image, callback)
    

    rospy.spin()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    receive_message()
