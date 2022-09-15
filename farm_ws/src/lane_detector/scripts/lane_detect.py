from tokenize import group
import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np

path  = '/home/bhavik/neonLabs-ai/personal/farmbot/deep_learning/Farm photos and videos/IMG20220722165418.jpg'

img = cv.imread(path)

b = img[:,:,0]
g = img[:,:,1]
r = img[:,:,2]

hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

h = hsv[:,:,0]
s = hsv[:,:,1]
v = hsv[:,:,2]

# create mask of h range so that we can filter out the green
mask = cv.inRange(hsv, (36, 0, 0), (86, 255,255))

# find out image of non green pixels
mask_inv = cv.bitwise_not(mask)
res_inv = cv.bitwise_and(img, img, mask=mask_inv)

# find lane lines
gray = cv.cvtColor(res_inv, cv.COLOR_BGR2GRAY)

# blur the image
blur = cv.GaussianBlur(gray, (5,5), 0)

# canny edge detection
edges = cv.Canny(blur, 50, 100, apertureSize=3)

# find lines
lines = cv.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=600, maxLineGap=25)

# blank image of copy img
blank = np.zeros_like(img)

# draw lines
for line in lines:
    x1, y1, x2, y2 = line[0]
    cv.line(blank, (x1, y1), (x2, y2), (255, 255, 255), 2)


# crop image from x : 1200 to 3500 and y from 1000 to 3000
crop = blank[1000:3000, 1200:3500]


# show images
plt.imshow(blank)
plt.title('Original Image')

plt.show()