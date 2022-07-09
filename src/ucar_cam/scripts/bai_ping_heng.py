#! /usr/bin/python2.7
# coding=utf-8
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class image_converter:
    def __init__(self):    
        # 创建cv_bridge，声明图像的发布者和订阅者
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
    def white_balance_1(self,img):
        # 读取图像
        r, g, b = cv2.split(img)
        r_avg = cv2.mean(r)[0]
        g_avg = cv2.mean(g)[0]
        b_avg = cv2.mean(b)[0]
        # 求各个通道所占增益
        k = (r_avg + g_avg + b_avg) / 3
        kr = k / r_avg
        kg = k / g_avg
        kb = k / b_avg
        r = cv2.addWeighted(src1=r, alpha=kr, src2=0, beta=0, gamma=0)
        g = cv2.addWeighted(src1=g, alpha=kg, src2=0, beta=0, gamma=0)
        b = cv2.addWeighted(src1=b, alpha=kb, src2=0, beta=0, gamma=0)
        balance_img = cv2.merge([b, g, r])
        return balance_img

    def callback(self,data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        cv_image_2 = self.white_balance_1(cv_image)

        # 显示Opencv格式的图像
            # cv2.imshow("Image window", cv_image_2)  
            # cv2.waitKey(3)

        # 再将opencv格式额数据转换成ros image格式的数据发布
    
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image_2, "bgr8"))

if __name__ == '__main__':
     # 初始化ros节点
    rospy.init_node("cv_bridge_test")
    rospy.loginfo("Starting cv_bridge_test node")
    image_converter()
    print("111")
    rospy.spin()
    print("222")

