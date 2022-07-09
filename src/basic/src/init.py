#!/usr/bin/env python


import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header
import numpy as np
import math


def talker():
    pub1 = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
    pub2 = rospy.Publisher('base_pose_ground_truth', Odometry, queue_size=1)
     
    rospy.init_node('initialpose_groundtruth', anonymous=True)
   
    rate = rospy.Rate(10)
    initial_pose = PoseWithCovarianceStamped()
    ground_truth = Odometry()
    
    initial_pose.pose.pose.position.x = -0.5
    initial_pose.pose.pose.position.y = 0
    initial_pose.pose.pose.position.z = 0 
    initial_pose.pose.pose.orientation.x = 0
    initial_pose.pose.pose.orientation.y = 0
    initial_pose.pose.pose.orientation.z = 0
    initial_pose.pose.pose.orientation.w = 1

    ground_truth.pose.pose.position.x = -0.5
    ground_truth.pose.pose.position.y = 0
    ground_truth.pose.pose.position.z = 0 
    ground_truth.pose.pose.orientation.x = 0
    ground_truth.pose.pose.orientation.y = 0
    ground_truth.pose.pose.orientation.z = 0
    ground_truth.pose.pose.orientation.w = 1
    while not rospy.is_shutdown():

        pub1.publish(initial_pose)
        pub2.publish(ground_truth)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
