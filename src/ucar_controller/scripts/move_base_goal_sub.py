#!/usr/bin/env python2.7
'''move_base_goal ROS Node'''
import rospy
from std_msgs.msg import String
import actionlib
import roslib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseActionGoal

#goal_point=([(-0.5, -5, 0.0)ucar, (0.0, 0.0, -0.01, 1)])
goal_point=[2.11,-2.69,0,2.1]
def goal_pose(pose):  # <2>
    goal_pose = MoveBaseGoal()
    qur=tf.transformations.quaternion_from_euler(0,0,pose[3])
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0]
    goal_pose.target_pose.pose.position.y = pose[1]
    goal_pose.target_pose.pose.position.z = pose[2]
    goal_pose.target_pose.pose.orientation.x = qur[0]
    goal_pose.target_pose.pose.orientation.y = qur[1]
    goal_pose.target_pose.pose.orientation.z = qur[2]
    goal_pose.target_pose.pose.orientation.w = qur[3]
    return goal_pose

def callback(data):
    '''move_base_goal Callback Function'''
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  # <3>
    print("car begin go!")
    client.wait_for_server()
    goal = goal_pose(goal_point)
    client.send_goal(goal)
    client.wait_for_server()

def listener():
    '''move_base_goal Subscriber'''
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('move_base_goal', anonymous=True)

    rospy.Subscriber("keda_car_star_tpc", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
