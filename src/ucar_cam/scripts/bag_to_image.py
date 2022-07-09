#! /usr/bin/python2.7
# coding=utf-8
import roslib
import rosbag
import rospy
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
 
rgb = '/home/tianbot/image_result'  #rgb path
bridge = CvBridge()
 
file_handle2 = open('/home/ucar/bag_data/rgb-stamp.txt', 'w')
 
with rosbag.Bag('/home/ucar/bag_data/2021-07-10-21-36-50.bag', 'r') as bag:
    for topic,msg,t in bag.read_messages():
        if topic == "/usb_cam/image_raw":   #rgb topic
            cv_image = bridge.imgmsg_to_cv2(msg,"bgr8")
            timestr = "%.6f" %  msg.header.stamp.to_sec()   #rgb time stamp
            image_name = timestr+ ".png"
            path = "rgb/" + image_name
            file_handle2.write(timestr + " " + path + '\n')
            cv2.imwrite(rgb + image_name, cv_image)
file_handle2.close()
