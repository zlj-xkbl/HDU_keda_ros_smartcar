#!/usr/bin/env python



import rospy
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry

class OdomAMCL:
    
    pub = rospy.Publisher('pose2d_odometry', Odometry, queue_size=1)

    def __init__(self):
       
	self.odom_combined = rospy.Subscriber('/pose2D', Pose2D, self.read_deal, queue_size=1)
        self.odom_amcl = []
        rospy.Timer(rospy.Duration(0.084), self.timer_callback)
 
        
    def read_deal(self, msg):
	self.last_recieved_stamp = rospy.Time.now()
        self.odom_amcl = Odometry()
        self.odom_amcl.header = rospy.Time.now()
        self.odom_amcl.header.frame_id = 'odom'
        self.odom_amcl.child_frame_id = 'base_link'
        self.odom_amcl.pose = msg.pose
        
        
        rospy.Timer(rospy.Duration(0.084), self.timer_callback)


    def timer_callback (self, even):
        #if self.last_recieved_stamp is None:
        #    return
        self.odom_amcl.header.stamp = self.last_recieved_stamp
        self.pub.publish(self.odom_amcl)

        
if __name__ == '__main__':
    try:
	rospy.init_node('odom_amcl', anonymous=False)
        OdomAMCL()
        rospy.spin()
    except:
        pass
        

        















