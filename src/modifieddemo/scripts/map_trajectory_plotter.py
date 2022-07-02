#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
import os
from modifieddemo.srv import SaveMap
import math

# Pose class
class Pose:
    def __init__(self, x, y):
        self.x = x
        self.y = y

cwd = os.getcwd()
image = None
imageInfo = None
path = []

''' 
    This Callback waits for the map occupancy grid and puts it into a format
    with color to draw the trajectory onto it
'''
def callbackMap(data):
    global imageInfo
    width = data.info.width
    height = data.info.height
    size =  (width , height)
    global image
    imageInfo = data.info
    image = np.zeros(size)
    
    counter = 0
    #print (len(data.data))
    for p in data.data:
      image[counter%width][int(counter/width)] = p
      counter+=1
    
    # Add color to the image
    img_float32 = np.float32(image)
    image = cv2.cvtColor(img_float32, cv2.COLOR_GRAY2RGB)
    image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    image[image == -1] = 150
    image[image == 0] = 255
    image[image == 100] = 0
    print("Map received!")


''' 
    This Callback waits for each PoseWithCovariance published by amcl on /amcl_pose topic
    by amcl localization, saves it into a an list and creates a line between the sencond-to-last and the last pose
    on the map image.
    Coordinate transformation from movement coordinates to image coordinates are
    carried out by the function transform_real_coords_to_img_coords
'''
def callbackPath(data):
    global image
    global imageInfo
    global path
    thickness = 2
    color = (0, 0, 255)
    pose = Pose(data.pose.pose.position.x, data.pose.pose.position.y)
    path.append(pose)

    print(path[-1].x,path[-1].y)
    print(imageInfo.origin.position.x,imageInfo.origin.position.y)
    if (len(path)>2):
        point1 = transform_real_coords_to_img_coords(path[-2].x, path[-2].y, imageInfo)
        point2 =  transform_real_coords_to_img_coords(path[-1].x, path[-1].y, imageInfo)
        image = cv2.line(image, point1, point2, color, thickness)

'''
    This function carries out 
'''
def transform_real_coords_to_img_coords(pose_x, pose_y, image_info):
    # Origin (0,0) is in the upper-left corner. 
    # Y-axis has opposite direction w.r.t. usual y coordinate
    # Origin of the robot is in the center on the map in real coords
    origin_x = image_info.origin.position.x
    origin_y = image_info.origin.position.y
    x =    pose_x - origin_x
    y =  - pose_y - origin_y # Y-axis has opposite direction
    off_x = x // image_info.resolution # Image resolution tells us how many meters per cell the map represent
    off_y = y // image_info.resolution
    return (int(off_x), int(off_y))

'''
    This service exposes the custom service SaveMap.srv that take as input a string 
    that represents the path to save the map to and names it grid.png
'''
def saver(req):
    global image
    filename = req.value+'/grid.png'
    print (req.value+'/grid.png'+ " saving image")
    cv2.imwrite (filename,image)
    return 200

def listener():
    print ("starting")
    rospy.init_node('map_saver', anonymous=False)

    rospy.Subscriber("/map", OccupancyGrid, callbackMap)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callbackPath)

    savemap = rospy.Service('saveMap', SaveMap, saver)
    
    rospy.spin()

if __name__ == '__main__':
    listener()