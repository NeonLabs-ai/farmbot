import rosbag
import os, sys, time
from sensor_msgs.msg import Image
import roslib
roslib.load_manifest('sensor_msgs')
from PIL import ImageFile
import matplotlib.pyplot as plt
import rospy

# path_to_imgs = sys.argv[0]
# bag_name = sys.argv[1]

path_to_imgs = '/home/bhavik/neonLabs-ai/personal/farmbot/deep_learning/dataset/test/images/'
bag_name = 'test.bag'

bag = rosbag.Bag(bag_name, 'w')
Stamp = rospy.Time()

# add all imgs to list
imgs = []
for file in os.listdir(path_to_imgs):
    if file.endswith(".jpg"):
        imgs.append(os.path.join(path_to_imgs, file))


for root, dirs, files in os.walk(path_to_imgs):
    files.sort()
    for file in files:
        if file.endswith(".jpg"):
            print("Adding image: " + file)
            img = Image.open(path_to_imgs + file)
            # img = plt.imread(os.path.join(root, file))

            Stamp = time.time()
            Stamp = rospy.Time.from_sec(Stamp)

            msg = Image()
            msg.header.stamp = Stamp
            msg.height = img.shape[0]
            msg.width = img.shape[1]
            msg.encoding = "rgb8"
            msg.is_bigendian = 0
            # msg.step = len(img.tobytes())
            # msg.data = img.tobytes()
            msg.data = [pix for pixdata in img.getdata() for pix in pixdata]
            
            bag.write('camera/image_raw', msg, t=Stamp)
            time.sleep(0.05)

bag.close()
        
    