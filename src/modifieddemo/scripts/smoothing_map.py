#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
import os
from modifieddemo.srv import SaveMap



cwd = os.getcwd()


def callbackMap(data):
    width = data.info.width
    height = data.info.height
    size =  (width , height)
    global image
    image = np.zeros(size)
    
    counter = 0
    #print (len(data.data))
    for p in data.data:
      image[counter%width][int(counter/width)] = p
      counter+=1
    
    
    
    kernel = np.ones((2,2),np.uint8)
    image = cv2.dilate(image,kernel,iterations = 1)
    image = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
    custom_map = OccupancyGrid()

    custom_map.header = data.header
    custom_map.info = data.info
    custom_map.data =  np.transpose(image).flatten().astype (int)



    image = cv2.rotate(image, cv2.ROTATE_180)
    image[image == -1] = 150
    image[image == 0] = 255
    image[image == 100] = 0
    print("AAAAAAAAAAAAAAAAAAAAAAAa")

def callbackPath(data):
    poses = data.poses

def saver(req):
    filename = cwd+'/grid.png'
    print (cwd+'/grid.png'+"saving image")
    cv2.imwrite (filename,image)
    return 200

def listener():
    print ("starting")
    rospy.init_node('depth_saver', anonymous=False)

    rospy.Subscriber("/map", OccupancyGrid, callbackMap)
    rospy.Subscriber("/trajectory", Path, callbackPath)

    savemap = rospy.Service('saveMap', SaveMap, saver)
    
    rospy.spin()

if __name__ == '__main__':
    listener()