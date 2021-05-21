#!/bin/python3
import sys
import cv2
import matplotlib.pyplot as plt
 
# get argument list using sys module
inp = sys.argv[1]

# Open image
try:
    img = cv2.imread(inp)
    ## convert to hsv
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    ## mask of blue (90,0,0) ~ (110, 255,255)
    mask1 = cv2.inRange(hsv, (110, 0, 0), (130, 255,255))
    target = cv2.bitwise_and(img,img, mask=mask1)
    mask1bgr = cv2.cvtColor(mask1, cv2.COLOR_RGB2BGR)
    targetbgr = cv2.cvtColor(target, cv2.COLOR_RGB2BGR)
    # plt.imshow(targetbgr)
    plt.imshow(mask1bgr)
    plt.show()
except Exception as e:
    print('Error!', e)
