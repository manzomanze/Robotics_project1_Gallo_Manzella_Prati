#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
import os
from modifieddemo.srv import SaveMap



cwd = os.getcwd()
image = None
imagesize = None

def callbackMap(data):
    global imagesize
    width = data.info.width
    height = data.info.height
    size =  (width , height)
    global image
    imagesize = data.info
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
    print(poses[-1].pose.position.x,poses[-1].pose.position.y)
    """ Point p1(0, 0)
    Point p2(100, 0)
    Point p3(200, 0) """ 
    


def saver(req):
    global image
    global imagesize
    p4 = (imagesize.height/2, imagesize.width/2)
    p2 = (20, 20)
    thickness = 2
    color = (0, 0, 255)
    
    image = cv2.line(image, p4, p2, color, thickness)

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