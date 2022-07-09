#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan
import geometry_msgs.msg
import tf2_ros
import tf
from std_msgs.msg import Int32,String
import math
import numpy as np
from peaple_data_msg.msg import peaple_spe
from darknet_ros_msgs.msg import BoundingBoxes
import random
import message_filters
#def wave_filtering(s):
#    mean = []
#    n = 3     #一次采样的次数
#    for i in range(int(len(s)/n)):
#     select_s = s[i*n:(i+1)*n]     #切片选取一次采样的个数
#     select_s = sorted(select_s)
#     select_s.remove(select_s[0])
#     select_s.remove(select_s[-1])
#     mean_s = np.mean(select_s)
#     mean.append(mean_s)
#    return np.mean(mean) #返回平均
     #return np.median(mean)#返回中位值
# def wave_filtering(s):
#     mean = []
#     select_s = sorted(s)
#     select_s_arr=[]
#     arr_dic=dict()
#     MAX_DATA=0
#     for i in range(len(s)):
#         select_s_arr.append(select_s[i]-select_s[i-1])
#         arr_dic[select_s[i]-select_s[i-1]]=i
#     select_s_arr=sorted(select_s_arr)
#     k=arr_dic[select_s_arr[len(select_s_arr)-1]]
#     m=arr_dic[select_s_arr[len(select_s_arr)-2]]
#     if select_s_arr[len(select_s_arr)-2]<0.05:
#         m=0
#     select_s=select_s[m:k]
#     return np.median(select_s)
def wave_filtering(s):
    #print("s:",s)
    sum=0
    select_s = sorted(s)
    select_s_arr=[]
    arr_dic=dict()
    MAX_DATA=0
    for i in range(len(s)):
        select_s_arr.append(select_s[i]-select_s[i-1])
        arr_dic[select_s[i]-select_s[i-1]]=i
    select_s_arr=sorted(select_s_arr)
    k=arr_dic[select_s_arr[len(select_s_arr)-1]]
    m=arr_dic[select_s_arr[len(select_s_arr)-2]]
    if select_s_arr[len(select_s_arr)-2]<0.05:
        m=0
    ## if m==k:
    ##     select_s_data=
    select_s=select_s[m:k]
    #print("select_s:",select_s)
    # for i in range(len(select_s)):
    #     sum=sum+select_s[i]
    return np.mean(select_s)


        # if abs(select_s[i+1]-select_s[i])>MAX_DATA:
        #     MAX_DATA=abs(select_s[i+1]-select_s[i])
    #     #     index_i=i
    # min_Data=select_s[0:index_i]
    # return np.mean(min_Data) #返回平均
    # #return np.median(mean)#返回中位值
def get_scan(angle):
    # self.scan_filter = []
    index=round(756/360*angle)
    #print("lider line long:",msg.ranges[index],"lider size ",index)
    return index                    
def Calculatecircle(detax,detay):
    return math.sqrt(detax**2+detay**2)
def Circle_cheak(Point_Arr_Female,Point_Arr_Glass,pair,hashmap1,hashmap2,ID):
    Circle=0.3
    flag=0
    if(ID==2):
        l=len(Point_Arr_Female)
        if(l>0):
            for i in range(len(Point_Arr_Female)):
                #print(Calculatecircle(abs(Point_Arr_Female[i][0]-pair[0]),abs(Point_Arr_Female[i][1]-pair[1])))
                if(Calculatecircle(abs(Point_Arr_Female[i][0]-pair[0]),abs(Point_Arr_Female[i][1]-pair[1]))>Circle):
                    flag=1
        if(l==0):
            flag=1
        if(flag==1):
            Point_Arr_Female.append(pair)
            hashmap1[pair]=1
    if(ID==0):
        l=len(Point_Arr_Glass)
        if(l>0):
            for i in range(len(Point_Arr_Glass)):
                print(Calculatecircle(abs(Point_Arr_Glass[i][0]-pair[0]),abs(Point_Arr_Glass[i][1]-pair[1])))
                if(Calculatecircle(abs(Point_Arr_Glass[i][0]-pair[0]),abs(Point_Arr_Glass[i][1]-pair[1]))>Circle):
                    flag=1
        if(l==0):
            flag==1
        if(flag==1):
            Point_Arr_Glass.append(pair)
            hashmap2[pair]=1

"""
Created on Tue Jul 13 14:12:20 2021

@author: Natmat
"""


def Transform_To_BLock(x,y):
    a=0#默认开始区块
    if(x<=0.2 and x>-0.3 and y<=0.3 and y>-0.2 ):
        a=1
    if(x<=0.7 and x>0.2 and y<=0.3 and y>-0.2 ):
        a=2
    if(x<=1.2 and x>0.7 and y<=0.3 and y>-0.2 ):
        a=3
    if(x<=1.7 and x>1.2 and y<=0.3 and y>-0.2 ):
        a=4
    if(x<=2.2 and x>1.7 and y<=0.3 and y>-0.2 ):
        a=5
    if(x<=2.7 and x>2.2 and y<=0.3 and y>-0.2 ): 
        a=6
    if(x<=3.2 and x>2.7 and y<=0.3 and y>-0.2 ):
        a=7
    if(x<=3.7 and x>3.2 and y<=0.3 and y>-0.2 ):
        a=8
    #第一列
    if(x<=0.2 and x>-0.3 and y<=-0.2 and y>-0.7 ):
        a=16
    if(x<=0.7 and x>0.2 and y<=-0.2 and y>-0.7 ):
        a=15
    if(x<=1.2 and x>0.7 and y<=-0.2 and y>-0.7 ):
        a=14
    if(x<=1.7 and x>1.2 and y<=-0.2 and y>-0.7 ):
        a=13
    if(x<=2.2 and x>1.7 and y<=-0.2 and y>-0.7 ):
        a=12
    if(x<=2.7 and x>2.2 and y<=-0.2 and y>-0.7 ):
        a=11
    if(x<=3.2 and x>2.7 and y<=-0.2 and y>-0.7 ):
        a=10
    if(x<=3.7 and x>3.2 and y<=-0.2 and y>-0.7 ):
        a=9
    #第二列
    if(x<=0.2 and x>-0.3 and y<=-0.7 and y>-1.2 ):
        a=17
    if(x<=0.7 and x>0.2 and y<=-0.7 and y>-1.2 ):
        a=18
    if(x<=1.2 and x>0.7 and y<=-0.7 and y>-1.2 ):
        a=19
    if(x<=1.7 and x>1.2 and y<=-0.7 and y>-1.2 ):
        a=20
    if(x<=2.2 and x>1.7 and y<=-0.7 and y>-1.2 ):
        a=21
    if(x<=2.7 and x>2.2 and y<=-0.7 and y>-1.2 ):
        a=22
    if(x<=3.2 and x>2.7 and y<=-0.7 and y>-1.2 ):
        a=23
    if(x<=3.7 and x>3.2 and removingy<=-0.7 and y>-1.2 ):
        a=24
    #第三列
    if(x<=0.2 and x>-0.3 and y<=-1.2 and y>-1.7 ):
        a=32
    if(x<=0.7 and x>0.2 and y<=-1.2 and y>-1.7 ):
        a=31
    if(x<=1.2 and x>0.7 and y<=-1.2 and y>-1.7 ):
        a=30
    if(x<=1.7 and x>1.2 and y<=-1.2 and y>-1.7 ):
        a=29
    if(x<=2.2 and x>1.7 and y<=-1.2 and y>-1.7 ):
        a=28
    if(x<=2.7 and x>2.2 and y<=-1.2 and y>-1.7 ):
        a=27
    if(x<=3.2 and x>2.7 and y<=-1.2 and y>-1.7 ):
        a=26
    if(x<=3.7 and x>3.2 and y<=-1.2 and y>-1.7 ):
        a=25
    #第四列
    if(x<=0.2 and x>-0.3 and y<=-1.7 and y>-2.2 ):
        a=33
    if(x<=0.7 and x>0.2 and y<=-1.7 and y>-2.2 ):
        a=34
    if(x<=1.2 and x>0.7 and y<=-1.7 and y>-2.2 ):
        a=35
    if(x<=1.7 and x>1.2 and y<=-1.7 and y>-2.2 ):
        a=36
    if(x<=2.2 and x>1.7 and y<=-1.7 and y>-2.2 ):
        a=37
    if(x<=2.7 and x>2.2 and y<=-1.7 and y>-2.2 ):
        a=38
    if(x<=3.2 and x>2.7 and y<=-1.7 and y>-2.2 ):
        a=39
    if(x<=3.7 and x>3.2 and y<=-1.7 and y>-2.2 ):
        a=40
    #第五列
    if(x<=0.2 and x>-0.3 and y<=-2.2 and y>-2.7 ):
        a=48
    if(x<=0.7 and x>0.2 and y<=-2.2 and y>-2.7 ):
        a=47
    if(x<=1.2 and x>0.7 and y<=-2.2 and y>-2.7 ):
        a=46
    if(x<=1.7 and x>1.2 and y<=-2.2 and y>-2.7 ):
        a=45
    if(x<=2.2 and x>1.7 and y<=-2.2 and y>-2.7 ):
        a=44
    if(x<=2.7 and x>2.2 and y<=-2.2 and y>-2.7 ):
        a=43
    if(x<=3.2 and x>2.7 and y<=-2.2 and y>-2.7 ):
        a=42
    if(x<=3.7 and x>3.2 and y<=-2.2 and y>-2.7 ):
        a=41
    #第六列
    if(x<=0.2 and x>-0.3 and y<=-2.7 and y>-3.2 ):
        a=49
    if(x<=0.7 and x>0.2 and y<=-2.7 and y>-3.2 ):
        a=50
    if(x<=1.2 and x>0.7 and y<=-2.7 and y>-3.2 ):
        a=51
    if(x<=1.7 and x>1.2 and y<=-2.7 and y>-3.2 ):
        a=52
    if(x<=2.2 and x>1.7 and y<=-2.7 and y>-3.2 ):
        a=53
    if(x<=2.7 and x>2.2 and y<=-2.7 and y>-3.2 ):
        a=54
    if(x<=3.2 and x>2.7 and y<=-2.7 and y>-3.2 ):
        a=55
    if(x<=3.7 and x>3.2 and y<=-2.7 and y>-3.2 ):
        a=56
    #第七列
    if(x<=0.2 and x>-0.3 and y<=-3.2 and y>-3.7 ):
        a=64
    if(x<=0.7 and x>0.2 and y<=-3.2 and y>-3.7 ):
        a=63                    
    if(x<=1.2 and x>0.7 and y<=-3.2 and y>-3.7 ):
        a=62
    if(x<=1.7 and x>1.2 and y<=-3.2 and y>-3.7 ):
        a=61
    if(x<=2.2 and x>1.7 and y<=-3.2 and y>-3.7 ):
        a=60
    if(x<=2.7 and x>2.2 and y<=-3.2 and y>-3.7 ):
        a=59
    if(x<=3.2 and x>2.7 and y<=-3.2 and y>-3.7 ):
        a=58
    if(x<=3.7 and x>3.2 and y<=-3.2 and y>-3.7 ):
        a=57
    #第八列
    return a
def Cheakmap(Block_number,map1,map2,arr):
        if (arr==0):
            if(map1[Block_number]==0):
                map1[Block_number]=map1[Block_number]+1
        if (arr==2):
            if(map2[Block_number]==0):
                map2[Block_number]=map2[Block_number]+1
        
def Getfemale_Sum(map2):
    female_Sum=0
    for key in map2:
        female_Sum=female_Sum+map2[key]
    return female_Sum
def Getglass_Sum(map1):
    glass_Sum=0
    for key in map1:
        glass_Sum=glass_Sum+map1[key]
    return glass_Sum
# x=2.2
# y=-3.2
# Block_number=Transform_To_BLock(x, y)
# #print(Block_number)
# arr=1
# map1={1:0,2:0,3:0,4:0,5:0,6:0,7:0,8:0,
#       9:0,10:0,11:0,12:0,13:0,14:0,15:0,16:0,
#       17:0,18:0,19:0,20:0,21:0,22:0,23:0,24:0,
#       25:0,26:0,27:0,28:0,29:0,30:0,31:0,32:0,
#       33:0,34:0,35:0,36:0,37:0,38:0,39:0,40:0,
#       41:0,42:0,43:0,44:0,45:0,46:0,47:0,48:0,
#       49:0,50:0,51:0,52:0,53:0,54:0,55:0,56:0,
#       57:0,58:0,59:0,60:0,61:0,62:0,63:0,64:0}                        if abs(self.scan_filter[i]-
# map2={1:0,2:0,3:0,4:0,5:0,6:0,7:0,8:0,
#       9:0,10:0,11:0,12:0,13:0,14:0,15:0,16:0,
#       17:0,18:0,19:0,20:0,21:0,22:0,23:0,24:0,
#       25:0,26:0,27:0,28:0,29:0,30:0,31:0,32:0,
#       33:0,34:0,35:0,36:0,37:0,38:0,39:0,40:0,
#       41:0,42:0,43:0,44:0,45:0,46:0,47:0,48:0,
#       49:0,50:0,51:0,52:0,53:0,54:0,55:0,56:0,
#       57:0,58:0,59:0,60:0,61:0,62:0,63:0,64:0}
map1={(0,0):0}
map2={(0,0):0}

# Cheakmap(Block_number, map1, map2, arr)
# female_Sum=Getfemale_Sum(map2)
# glass_Sum=Getglass_Sum(map1)
# print(female_Sum,glass_Sum)
#map1存眼睛，map2存女性及长头发
"""
hashmap={0:0,1:0}
hashmap1={0:0,1:0}
hashmap1[0]=hashmap1[0]+2
print(hashmap1[0])
"""    
class people_location():
    def __init__(self):
        self.LIDAR_ERR = 1.5
        # self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        #self.people_location()
        self.angle=0
        self.distance=0
        self.pose_x=0
        self.pose_y=0
        self.pose_x_in_map=0
        self.pose_y_in_map=0
        self.long_hair=0
        self.glass=0
        self.scan_filter=[]
        self.x_list=[]
        self.ang_data_list=[]
        self.ang_data=255
        self.LIDAR_MIN_ERR=0.05
        self.marker_pose=geometry_msgs.msg.TransformStamped()
        self.Point_Arr_Female=[]
        self.Point_Arr_Glass=[]
        self.arr_angel=dict()
        #self.Point_Arr=[]
    def callback(self,scan_info,peopel_info):
        people_len=len(peopel_info.bounding_boxes)
        #print("Block_number:",Block_number)
        #listener.waitForTransform('/map',self.marker_pose.child_frame_id,rospy.Time(0),rospy.Duration(4.0))
        #print("people_len:",people_len)
        for i in range(people_len):
            if peopel_info.bounding_boxes[i].id==0:#galss
                self.x_list.append(round((peopel_info.bounding_boxes[i].xmin+peopel_info.bounding_boxes[i].xmax)/2))               
            # elif msg.bounding_boxes[i].id==1:#boy
            #     self.x_list.append(round((msg.bounding_boxes[i].xmin+msg.bounding_boxes[i].xmax)/2))
            elif peopel_info.bounding_boxes[i].id==2:#girl
                self.x_list.append(round((peopel_info.bounding_boxes[i].xmin+peopel_info.bounding_boxes[i].xmax)/2))
        for i in range(people_len):
            if self.x_list[i]>=0 and self.x_list[i]<160:
                self.ang_data_list.append(0)
            elif self.x_list[i]>=160 and self.x_list[i]<320:
                self.ang_data_list.append(1)
            elif self.x_list[i]>=320 and self.x_list[i]<480:
                self.ang_data_list.append(2)
            elif self.x_list[i]>=480 and self.x_list[i]<640:
                self.ang_data_list.append(3)
            elif self.x_list[i]>=640 and self.x_list[i]<800:
                self.ang_data_list.append(4)
        self.x_list=[]
        if len(self.ang_data_list)>0:
            for i in range(len(self.ang_data_list)):
                self.ang_data=self.ang_data_list[i]
                for angle in range(360):
                    if self.ang_data==4 and angle >=192-4  and angle <= 222-12:
                        self.angle_base= (192+222)/2-180         
                        if scan_info.ranges[int(get_scan(angle))] <= self.LIDAR_ERR and scan_info.ranges[int(get_scan(angle))] >=self.LIDAR_MIN_ERR:
                            self.scan_filter.append(scan_info.ranges[int(get_scan(angle))])
                            self.arr_angel[scan_info.ranges[int(get_scan(angle))]]=angle-180
                    elif self.ang_data==3 and angle >=180 and angle <= 207:
                        self.angle_base= (180+207)/2 -180       
                        if scan_info.ranges[int(get_scan(angle))] <= self.LIDAR_ERR and scan_info.ranges[int(get_scan(angle))] >=self.LIDAR_MIN_ERR:
                            self.scan_filter.append(scan_info.ranges[int(get_scan(angle))])
                            self.arr_angel[scan_info.ranges[int(get_scan(angle))]]=angle-180    
                    elif self.ang_data==2 and angle >=168  and angle <= 192:
                        self.angle_base=(168+192)/2-180          
                        if scan_info.ranges[int(get_scan(angle))] <= self.LIDAR_ERR and scan_info.ranges[int(get_scan(angle))] >=self.LIDAR_MIN_ERR:
                            self.scan_filter.append(scan_info.ranges[int(get_scan(angle))])
                            self.arr_angel[scan_info.ranges[int(get_scan(angle))]]=angle-180         
                    elif self.ang_data==1 and angle >=153  and angle <= 180: 
                        self.angle_base=(153+180)/2-180          
                        if scan_info.ranges[int(get_scan(angle))] <= self.LIDAR_ERR and scan_info.ranges[int(get_scan(angle))] >=self.LIDAR_MIN_ERR:
                            self.scan_filter.append(scan_info.ranges[int(get_scan(angle))])
                            self.arr_angel[scan_info.ranges[int(get_scan(angle))]]=angle-180    
                    elif self.ang_data==0 and angle >=138+4  and angle <= 163+7:
                        self.angle_base= (138+163)/2-180         
                        if scan_info.ranges[int(get_scan(angle))] <= self.LIDAR_ERR and scan_info.ranges[int(get_scan(angle))] >=self.LIDAR_MIN_ERR:
                            self.scan_filter.append(scan_info.ranges[int(get_scan(angle))])
                            self.arr_angel[scan_info.ranges[int(get_scan(angle))]]=angle-180
                            
                                
                if self.ang_data>=0 and self.ang_data<=4:
                    (trans,rot) = listener.lookupTransform('/map','/base_link',rospy.Time(0))
                    yaw=tf.transformations.euler_from_quaternion(rot)
                    #print(yaw)
                    self.distance=wave_filtering(self.scan_filter)
                    #print("distance:",self.distance)
                    #scan_filter_s=sorted(self.scan_filter)
                    angle_lis=[]
                    ANG_MIN=255
                    for i in range(len(self.scan_filter)):
                        if abs(self.scan_filter[i]-self.distance)<0.1:
                            angle_lis.append(self.arr_angel[self.scan_filter[i]])
                    # for i in range(len(angle_lis)):
                    #     if abs(self.angle_base-angle_lis[i])<ANG_MIN:
                    #         ANG_MIN=abs(self.angle_base-angle_lis[i])
                    #         self.angle=angle_lis[i]
                    print("angel_lis:",angle_lis)
                    self.angle=np.mean(angle_lis)
                    angle_to_pi=(2*math.pi*self.angle)/360
                    self.pose_x=self.distance*math.cos(angle_to_pi)
                    self.pose_y=self.distance*math.sin(angle_to_pi)
                    print("x:",self.pose_x,"y:",self.pose_y)
                    qur=tf.transformations.quaternion_from_euler(0,0,1)
                    self.marker_pose.transform.rotation.x = qur[0]
                    self.marker_pose.transform.rotation.y = qur[1]
                    self.marker_pose.transform.rotation.z = qur[2]
                    self.marker_pose.transform.rotation.w = qur[3]
                    self.marker_pose.transform.translation.x = self.pose_x
                    self.marker_pose.transform.translation.y = self.pose_y
                    self.marker_pose.transform.translation.z = 0 
                    self.marker_pose.header.stamp = rospy.Time.now()
                    self.marker_pose.header.frame_id = "laser_frame"
                    self.marker_pose.child_frame_id = "people_pose_"+str(self.ang_data)#str(round(random.randint(1,10)))
                    self.marker_pose.transform = self.marker_pose.transform
                    tfcaster.sendTransform(self.marker_pose)                                            
                    self.scan_filter=[]
                    self.arr_angel.clear()

                    (trans,rot)=listener.lookupTransform('/map',self.marker_pose.child_frame_id,rospy.Time(0))
                    self.pose_x_in_map=trans[0]
                    self.pose_y_in_map=trans[1]
                    pair=(self.pose_x_in_map,self.pose_y_in_map)
                    #print("pair:",pair)
                    Circle_cheak(self.Point_Arr_Female,self.Point_Arr_Glass,pair,map1,map2,peopel_info.bounding_boxes[i].id)#map1->female map2->glass
                    female_Sum=Getfemale_Sum(map1)
                    glass_Sum=Getglass_Sum(map2)
                    print("female_Sum:",female_Sum,"glass_Sum:",glass_Sum)
                    #(trans,rot)=listener.waitForTransform('/map',self.marker_pose.child_frame_id,rospy.Time(0),rospy.Duration(4.0))
                    # (trans,rot)=listener.lookupTransform('/map',self.marker_pose.child_frame_id,rospy.Time(0))
                    # self.pose_x_in_map=trans[0]
                    # self.pose_y_in_map=trans[1]
                    # #print(female_Sum,glass_Sum)
                    # print("x_inmap:",trans[0],"y_inmap:",trans[1])    
        self.ang_data_list=[]
        # if self.ang_data==0:
        #     if get_scan(i) <= "inf":
        # Block_number=Transform_To_BLock(self.pose_x_in_map, self.pose_y_in_map)
        # Cheakmap(Block_number, map1, map2, id)
        #listener.waitForTransform('/map',self.marker_pose.child_frame_id,rospy.Time(0),rospy.Duration(4.0))
        (trans,rot)=listener.lookupTransform('/map',self.marker_pose.child_frame_id,rospy.Time(0))
        self.pose_x_in_map=trans[0]
        self.pose_y_in_map=trans[1]
        #print("x_in_map:",trans[0],"y_in_map:",trans[1])
        
        pair=(self.pose_x_in_map,self.pose_y_in_map)
        #print("pair:",pair)
        Circle_cheak(self.Point_Arr_Female,self.Point_Arr_Glass,pair,map1,map2,peopel_info.bounding_boxes[i].id)#map1->female map2->glass
        female_Sum=Getfemale_Sum(map1)
        glass_Sum=Getglass_Sum(map2)
        print("female_Sum:",female_Sum,"glass_Sum:",glass_Sum)
    def scan_cb(self,msg):
        # (trans,rot) = listener.lookupTransform('/map','/base_link',rospy.Time(0))
        # yaw=tf.transformations.euler_from_quaternion(rot)
        # print(yaw)
        if len(self.ang_data_list)>0:
            for i in range(len(self.ang_data_list)):
                self.ang_data=self.ang_data_list[i]
                for angle in range(360):
                    if self.ang_data==4 and angle >=192-4  and angle <= 222-12:
                        self.angle_base= (192+222)/2-180         
                        if msg.ranges[int(get_scan(angle))] <= self.LIDAR_ERR and msg.ranges[int(get_scan(angle))] >=self.LIDAR_MIN_ERR:
                            self.scan_filter.append(msg.ranges[int(get_scan(angle))])
                            self.arr_angel[msg.ranges[int(get_scan(angle))]]=angle-180
                    elif self.ang_data==3 and angle >=180 and angle <= 207:
                        self.angle_base= (180+207)/2 -180       
                        if msg.ranges[int(get_scan(angle))] <= self.LIDAR_ERR and msg.ranges[int(get_scan(angle))] >=self.LIDAR_MIN_ERR:
                            self.scan_filter.append(msg.ranges[int(get_scan(angle))])
                            self.arr_angel[msg.ranges[int(get_scan(angle))]]=angle-180    
                    elif self.ang_data==2 and angle >=168  and angle <= 192:
                        self.angle_base=(168+192)/2-180          
                        if msg.ranges[int(get_scan(angle))] <= self.LIDAR_ERR and msg.ranges[int(get_scan(angle))] >=self.LIDAR_MIN_ERR:
                            self.scan_filter.append(msg.ranges[int(get_scan(angle))])
                            self.arr_angel[msg.ranges[int(get_scan(angle))]]=angle-180         
                    elif self.ang_data==1 and angle >=153  and angle <= 180: 
                        self.angle_base=(153+180)/2-180          
                        if msg.ranges[int(get_scan(angle))] <= self.LIDAR_ERR and msg.ranges[int(get_scan(angle))] >=self.LIDAR_MIN_ERR:
                            self.scan_filter.append(msg.ranges[int(get_scan(angle))])
                            self.arr_angel[msg.ranges[int(get_scan(angle))]]=angle-180    
                    elif self.ang_data==0 and angle >=138+4  and angle <= 163+7:
                        self.angle_base= (138+163)/2-180         
                        if msg.ranges[int(get_scan(angle))] <= self.LIDAR_ERR and msg.ranges[int(get_scan(angle))] >=self.LIDAR_MIN_ERR:
                            self.scan_filter.append(msg.ranges[int(get_scan(angle))])
                            self.arr_angel[msg.ranges[int(get_scan(angle))]]=angle-180
                            
                                
                if self.ang_data>=0 and self.ang_data<=4:
                    (trans,rot) = listener.lookupTransform('/map','/base_link',rospy.Time(0))
                    yaw=tf.transformations.euler_from_quaternion(rot)
                    #print(yaw)
                    self.distance=wave_filtering(self.scan_filter)
                    #print("distance:",self.distance)
                    #scan_filter_s=sorted(self.scan_filter)
                    angle_lis=[]
                    ANG_MIN=255
                    for i in range(len(self.scan_filter)):
                        if abs(self.scan_filter[i]-self.distance)<0.1:
                            angle_lis.append(self.arr_angel[self.scan_filter[i]])
                    # for i in range(len(angle_lis)):
                    #     if abs(self.angle_base-angle_lis[i])<ANG_MIN:
                    #         ANG_MIN=abs(self.angle_base-angle_lis[i])
                    #         self.angle=angle_lis[i]
                    print("angel_lis:",angle_lis)
                    self.angle=np.mean(angle_lis)
                    angle_to_pi=(2*math.pi*self.angle)/360
                    self.pose_x=self.distance*math.cos(angle_to_pi)
                    self.pose_y=self.distance*math.sin(angle_to_pi)
                    print("x:",self.pose_x,"y:",self.pose_y)
                    qur=tf.transformations.quaternion_from_euler(0,0,1)
                    self.marker_pose.transform.rotation.x = qur[0]
                    self.marker_pose.transform.rotation.y = qur[1]
                    self.marker_pose.transform.rotation.z = qur[2]
                    self.marker_pose.transform.rotation.w = qur[3]
                    self.marker_pose.transform.translation.x = self.pose_x
                    self.marker_pose.transform.translation.y = self.pose_y
                    self.marker_pose.transform.translation.z = 0 
                    self.marker_pose.header.stamp = rospy.Time.now()
                    self.marker_pose.header.frame_id = "laser_frame"
                    self.marker_pose.child_frame_id = "people_pose_"+str(self.ang_data)#str(round(random.randint(1,10)))
                    self.marker_pose.transform = self.marker_pose.transform
                    tfcaster.sendTransform(self.marker_pose)                                            
                    self.scan_filter=[]
                    self.arr_angel.clear()
                    #(trans,rot)=listener.waitForTransform('/map',self.marker_pose.child_frame_id,rospy.Time(0),rospy.Duration(4.0))
                    # (trans,rot)=listener.lookupTransform('/map',self.marker_pose.child_frame_id,rospy.Time(0))
                    # self.pose_x_in_map=trans[0]
                    # self.pose_y_in_map=trans[1]
                    # #print(female_Sum,glass_Sum)
                    # print("x_inmap:",trans[0],"y_inmap:",trans[1])    
        self.ang_data_list=[]
        # if self.ang_data==0:
        #     if get_scan(i) <= "inf":
        #             self.scan_filter.append(msg.ranges[i]))
        # if self.ang_data==0:
        # range=msg.ranges[round(756/360*angle],
    # def calculate_distance(self,lider_dis,thita):
    #     b=self.L_0
    #     thita
    #     c=get_scan()
    #     distance=0.5*(2*)
        
        # for i in range(360):
        #     if i <= 15 or i > 335:
        #         if msg.ranges[i] >= self.LIDAR_ERR:
        #             self.scan_filter.append(msg.ranges[i]))
    def ang_cb(self,msg):
        self.ang_data=msg.data
    def peopel_info_cb(self,msg):
        people_len=len(msg.bounding_boxes)
        #print("Block_number:",Block_number)
        #listener.waitForTransform('/map',self.marker_pose.child_frame_id,rospy.Time(0),rospy.Duration(4.0))
        #print("people_len:",people_len)
        for i in range(people_len):
            if msg.bounding_boxes[i].id==0:#galss
                self.x_list.append(round((msg.bounding_boxes[i].xmin+msg.bounding_boxes[i].xmax)/2))               
            # elif msg.bounding_boxes[i].id==1:#boy
            #     self.x_list.append(round((msg.bounding_boxes[i].xmin+msg.bounding_boxes[i].xmax)/2))
            elif msg.bounding_boxes[i].id==2:#girl
                self.x_list.append(round((msg.bounding_boxes[i].xmin+msg.bounding_boxes[i].xmax)/2))
        for i in range(people_len):
            if self.x_list[i]>=0 and self.x_list[i]<160:
                self.ang_data_list.append(0)
            elif self.x_list[i]>=160 and self.x_list[i]<320:
                self.ang_data_list.append(1)
            elif self.x_list[i]>=320 and self.x_list[i]<480:
                self.ang_data_list.append(2)
            elif self.x_list[i]>=480 and self.x_list[i]<640:
                self.ang_data_list.append(3)
            elif self.x_list[i]>=640 and self.x_list[i]<800:
                self.ang_data_list.append(4)
        self.x_list=[]
        # Block_number=Transform_To_BLock(self.pose_x_in_map, self.pose_y_in_map)
        # Cheakmap(Block_number, map1, map2, id)
        #listener.waitForTransform('/map',self.marker_pose.child_frame_id,rospy.Time(0),rospy.Duration(4.0))
        (trans,rot)=listener.lookupTransform('/map',self.marker_pose.child_frame_id,rospy.Time(0))
        self.pose_x_in_map=trans[0]
        self.pose_y_in_map=trans[1]
        #print("x_in_map:",trans[0],"y_in_map:",trans[1])
        
        pair=(self.pose_x_in_map,self.pose_y_in_map)
        #print("pair:",pair)
        Circle_cheak(self.Point_Arr_Female,self.Point_Arr_Glass,pair,map1,map2,msg.bounding_boxes[i].id)#map1->female map2->glass
        female_Sum=Getfemale_Sum(map1)
        glass_Sum=Getglass_Sum(map2)
        print("female_Sum:",female_Sum,"glass_Sum:",glass_Sum)
        #print("map2:",map1)
    # def people_location(self):
    #     self.twist = Twist()
    #     while not rospy.is_shutdown():
    #         self.get_scan(12)

            # if min(self.scan_filter) < 0.5:
            #     self.twist.linear.x = 0.0
            #     self.twist.angular.z = 0.0
            #     self._cmd_pub.publish(self.twist)
            #     rospy.loginfo('Stop!')

            # else:
            #     self.twist.linear.x = 0.0
            #     self.twist.angular.z = 0.0
            #     rospy.loginfo('distance of the obstacle : %f', min(self.scan_filter))

            # self._cmd_pub.publish(self.twist)

if __name__ == '__main__':
    rospy.init_node('lider_location')
    people_location = people_location()
    listener_map_people=tf.TransformListener()
    # goal_pub=rospy.Publisher("/people_angle",Int32,queue_size=1)
    # loc_cb_ = rospy.Subscriber("/people_angle",Int32, people_location.ang_cb)
    # scan_cb_ = rospy.Subscriber("/scan",LaserScan, people_location.scan_cb)
    # people_inf_cb_ = rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes, people_location.peopel_info_cb)

    scan_cb_ = message_filters.Subscriber("/scan",LaserScan)
    people_inf_cb_ = message_filters.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes)
    ts =message_filters. TimeSynchronizer([scan_cb_,people_inf_cb_],12)
    ts.registerCallback(people_location.callback)
    
    people_data_pub = rospy.Publisher('/peaple_data_to_last', peaple_spe, queue_size=1)
    tfcaster = tf2_ros.TransformBroadcaster()
    listener = tf.TransformListener()
    rospy.spin()
