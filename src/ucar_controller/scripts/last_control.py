#! /usr/bin/python2.7
# coding=utf-8
import math
import actionlib
import dynamic_reconfigure.client
import rosparam
import rospy
import tf
from geometry_msgs.msg import TransformStamped,Twist,PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseActionGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32,String
from peaple_data_msg.msg import peaple_spe #自定义消息类型，需要移植peaple_data_msg工作包
from ucar_controller.srv import Play_TTS_srv
# [position.x position.y position.z orientation.x orientation.y orientation.z orientation.w]
goal_point=[
        [2.15 ,-2.64,0,3.00], #2.16 -2.63
    #[2.25,-3.0,0,2.0],#第一阶段终点
    #[1.70,-3.18,0,-0.6],#第一阶段终点
    [0.93,-1,0,1.7],#第二阶段第一个终点
    [0.5,-1,0,1.7],#第二阶段第二个终点
    [0.07,-0.9,0,1.57],#第二阶段第三个终点

    [0.5,-1.8,0,1.7]#特殊点位
    ]
key_points_bata=[
    # [0,-2.65],
    [2.15,-2.7],#第一个关键点
    [0.5,-1.8] #第二个关键点
]


def is_passed(now_pos, next_waypoint,pose_index=0.2):
    dis_to_next_point = math.sqrt(
        (now_pos[0]-next_waypoint[0])**2+(now_pos[1]-next_waypoint[1])**2)
    #print('--dis:%.2f'%(dis_to_next_point))
    if dis_to_next_point <= pose_index:
        return True
    else:
        return False
def distance(point_1_x,point_1_y,point_2_x,point_2_y):
    dis = math.sqrt(
        (point_1_x-point_2_x)**2+(point_1_y-point_2_y)**2)
    return dis
def goal_pose(pose):  
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0]
    goal_pose.target_pose.pose.position.y = pose[1]
    goal_pose.target_pose.pose.position.z = pose[2]
    goal_pose.target_pose.pose.orientation.x = pose[3]
    goal_pose.target_pose.pose.orientation.y = pose[4]
    goal_pose.target_pose.pose.orientation.z = pose[5]
    goal_pose.target_pose.pose.orientation.w = pose[6]
    return goal_pose
def goal_pose_stp(pose):
    goal_pose=PoseStamped()
    qur=tf.transformations.quaternion_from_euler(0,0,pose[3])
    goal_pose.header.frame_id = 'map'
    goal_pose.pose.position.x = pose[0]
    goal_pose.pose.position.y = pose[1]
    goal_pose.pose.position.z = pose[2]
    goal_pose.pose.orientation.x = qur[0]
    goal_pose.pose.orientation.y = qur[1]
    goal_pose.pose.orientation.z = qur[2]
    goal_pose.pose.orientation.w = qur[3]
    return goal_pose
class laset_control:
    def __init__(self):
        self.arcuo_x = 0
        self.arcuo_y = 0
        self.arcuo_z = 0
        self.arcuo_id=""
        self.key_arcuo_id=255
        self.flag_arcuo_id=0
        self.stop_arcuo_id=0
        self.flag_mic=0
        self.pose_x=255
        self.pose_y=255
        self.pose_z=255
        self.last_pose_index=0.5#判断到达终点阙值
        self.long_hair_data=0
        self.meganei_data=0
        self.mic_flag=0
        self.mic_out_data=""
        self.caiping_data=""
        #self.goal
        self.reconfig_client_pp= dynamic_reconfigure.client.Client('/Pure_Pursuit')
    def get_aruco_pose(self, msg):#二维码回调函数
       #listener.waitForTransform('/map','/base_link',rospy.Time(0),rospy.Duration(6.0))
        (trans,rot) = listener_map_base.lookupTransform('/map','/base_link',rospy.Time(0))
        self.pose_x=trans[0]
        self.pose_y=trans[1]
        self.pose_z=trans[2]
        # print("pose x:",self.pose_x,"pose_y:",self.pose_y)
        # self.arcuo_x = msg.transform.translation.x
        # self.arcuo_y = msg.transform.translation.y
        # self.arcuo_z = msg.transform.translation.z
        #路过第一个关键点
        if is_passed((self.pose_x,self.pose_y),(key_points_bata[0][0],key_points_bata[0][1]),0.4) and (msg.child_frame_id =="marker_id0" or msg.child_frame_id =="marker_id1" or msg.child_frame_id =="marker_id2") and self.flag_arcuo_id==0:
            self.flag_arcuo_id=1#锁死二维码
            self.arcuo_id = msg.child_frame_id#获取菜品信息
            listener_map_arcuo.waitForTransform('/map',self.arcuo_id,rospy.Time(0),rospy.Duration(6.0))
            print("aruco_id :",self.arcuo_id)
            # client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  # 更新终点坐标
            # client.wait_for_server()
            if(self.arcuo_id == "marker_id0"):#TODO：终点坐标需要修改 没有映射关系，且终点停止的二维码映射关系也要修改
                goal = goal_pose_stp(goal_point[1])#前往终点1
                self.caiping_data="蔬菜"#菜品1名称信息
                self.stop_arcuo_id="marker_id0"
            elif(self.arcuo_id=="marker_id1"):
                goal = goal_pose_stp(goal_point[1])#前往终点2
                self.caiping_data="水果"#菜品2名称信息
                self.stop_arcuo_id="marker_id1"
            elif (self.arcuo_id=="marker_id2"):
                goal = goal_pose_stp(goal_point[1])#前往终点3
                self.caiping_data="肉类"#菜品3名称信息
                self.stop_arcuo_id="marker_id2"
            caiping_bobao="U-CAR本次运输的菜品是"+self.caiping_data
            #soujiu="开始搜救"
            print(caiping_bobao)
            params_pp={'Vcmd': 0,'steering_gain':0}
            config_pp=self.reconfig_client_pp.update_configuration(params_pp)
            rospy.wait_for_service('/xf_mic_tts_offline_node/play_txt_wav')#发布语音合成服务
            mic_data = rospy.ServiceProxy('/xf_mic_tts_offline_node/play_txt_wav', Play_TTS_srv)
            response = mic_data(caiping_bobao,"")
            twist=Twist()
            twist.angular.z = 0
            twist.linear.x = 0
            twist.linear.y = 0
            vel_pub.publish(twist)
            if(response.result=='ok'):
                params_pp={'Vcmd': 0.7,'steering_gain':0.6}# 0.8 0.6
                config_pp=self.reconfig_client_pp.update_configuration(params_pp)                
                print("we get menu and go to goal")
                #goal = goal_pose_stp(goal_point[4])#TODO 改成终点
                for j in range(5):
                    goal_pub.publish(goal)#TODO 可以改成终点
                    #goal_pub.publish(goal)
            # client.send_goal(goal)
            # client.wait_for_server()
        #路过第二个关键点,开始二维码辅助泊车
        if is_passed((self.pose_x,self.pose_y),(key_points_bata[1][0],key_points_bata[1][1]),1.0) and self.flag_arcuo_id==1:#原先是1
            print("at second point!")
            (trans,rot) = listener_map_arcuo.lookupTransform('/map',self.stop_arcuo_id,rospy.Time(0))#将arcuo坐标系转换到map坐标系下，arucoid可能需要修改为tf坐标位置节点
            self.arcuo_x = trans[0]#存储坐标位置
            self.arcuo_y = trans[1]
            self.arcuo_z = trans[2]
 #           print("dis:%lf ",distance(self.arcuo_x,self.pose_x,self.arcuo_y,self.pose_y))
            #print("x:%lf y:%lf",self.arcuo_x,self.pose_x,self.arcuo_y,self.pose_y,self.arcuo_z,self.pose_z)
 #           if is_passed((self.pose_x,self.pose_y),(self.arcuo_x,self.arcuo_y),self.arcuo_pose_a) :#车辆坐标点如果小于阙值,则发布小车停止命令

    def get_imc_pose(self,msg):#麦克风语音唤醒回调函数
        if msg.data>=0 and self.mic_flag==0:
            self.mic_flag=1
            # client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  # 第一次更新终点坐标 前往第一段终点
            # client.wait_for_server()
            goal = goal_pose_stp(goal_point[0])#前往第一段终点
            for j in range(5):
                goal_pub.publish(goal)
            # client.send_goal(goal)
            # client.wait_for_server()
            print("car star move!")
    def get_peaple_spe(self,msg):#人物识别回调函数
        self.long_hair_data=msg.long_hair
        self.meganei_data=msg.meganei
        print("long_hair_data:",self.long_hair_data,"meganei_data:",self.meganei_data)
    def goal_reach_data_cb(self,msg):#判断终点回调函数
        #if msg.data=="goal reach" and is_passed((self.pose_x,self.pose_y),(self.arcuo_x,self.arcuo_y),self.last_pose_index):
        twist=Twist()
        twist.angular.z = 0
        twist.linear.x = 0
        twist.linear.y = 0
        vel_pub.publish(twist)
        if self.flag_mic==0:#最终发布语音合成消息
            self.flag_mic=1
            self.mic_out_data="您的菜品已送达，请您取餐。识别到长头发"+str(self.long_hair_data)+"人,戴眼镜"+str(self.meganei_data)+"人"
            #soujiu_jieguo="发现被困人员2人"
            print( self.mic_out_data)
            rospy.wait_for_service('/xf_mic_tts_offline_node/play_txt_wav')#发布语音合成服务
            mic_data = rospy.ServiceProxy('/xf_mic_tts_offline_node/play_txt_wav', Play_TTS_srv)
            response = mic_data(self.mic_out_data,"")#TODO：因为太吵暂时注释
if __name__ == '__main__':
    rospy.init_node('last_controll')
    listener_map_base = tf.TransformListener()
    listener_map_arcuo=tf.TransformListener()
    goal_pub=rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=1)
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    laset_control = laset_control()
    listener_map_base.waitForTransform('/map','/base_link',rospy.Time(0),rospy.Duration(6.0))
    #listener_map_arcuo.waitForTransform('/map',laset_control.arcuo_id,rospy.Time(0),rospy.Duration(6.0))
    # (trans,rot) = listener.lookupTransform('/map','/base_link',rospy.Time(0))
    # laset_control.pose_x=trans[0]
    # laset_control.pose_y=trans[1]
    aruco_pose = rospy.Subscriber("/simple_aruco_detector/transforms", TransformStamped, laset_control.get_aruco_pose)#获取aruco码识别内容
    imc_cb = rospy.Subscriber("/mic/awake/angle", Int32, laset_control.get_imc_pose)#获取小车唤醒
    peple_data = rospy.Subscriber("/peaple_data_to_last", peaple_spe, laset_control.get_peaple_spe)#获取人物识别模型数据
    goal_reach_data = rospy.Subscriber("/goal_reach_data", String, laset_control.goal_reach_data_cb)#获取终点确认

    #voiceWords_pub=rospy.Publisher('/voiceWords', String, queue_size=5)
    rospy.spin()
    # except KeyboardInterrupt:
    #     print('操作已取消')
    #     exit(0)
