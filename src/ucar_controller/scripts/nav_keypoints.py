#! /usr/bin/python2.7
# coding=utf-8
import math
import actionlib
import dynamic_reconfigure.client
import rosparam
import rospy
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped,Twist
from std_msgs.msg import String
from ucar_controller.srv import Play_TTS_srv
from peaple_data_msg.msg import peaple_spe
# [x,y,yaw,max_vel_x,acc_lim_theta]
# key_points = [
#     [3.511233, -0.000090, 0.000, 1.0,1.3],  # 1
#     [4.042557, -1.565172, 3.082496, 0.9,1.3],  # 2
#     [2.936867, -1.032038, -3.14, 1.2,1.3],  # 3
#     [1.995490, -1.044512, -2.983582, 0.9,1.5],  # 4
#     # [4.646070, -1.056370, -2.934374,1.3],  # 5
#     # [2.218460, -2.536215, -1.542630,1.3],  # 6
#     # [0.661139, -3.180960, -1.812663,1.3],  # 7
#     # [4.646070, -1.056370, -2.934374,1.3],  # 8
#     [1.504895, -4.168083, 0.000, 1.3,1.3],  # 9
#     [2.956114, -4.201780, 0.000, 0.9,1.5],  # 10
#     [4.295692, -3.125136, 0.000, 1.0,1.5],  # 11
#     [4.627578, -5.242858, -2.392219, 2.0,1.8],  # 12
#     [1.054753, -5.769012, 2.688439, 1.0,1.3],  # 13 teb中为防止终点震荡单独附加一个限速点
#     [-0.2504603767395, -5.2709980011, 2.446521, 0.0,0]  # 14 终点
# ]
# [x,y,yaw,max_vel_x,acc_lim_theta,Vcmd,steering_gain]
key_points = [
    [1.60,-0.5,0,1.0,0.05,0.9,1.03],#1
    [2.7,0.0,0,1.0,0.05,0.7,0.9],#2
    #[3.0,-0.3,0,1.0,0.2,1.2,1.2],#3
 
    #[3.0,-1.10,0,1.0,0.1,1.0,1.4],#3
    [2.7,-1.50,0,1.0,0.1,0.9,0.7],#3CCCcC
    [2.2, -1.45, 0.0,1.0,1.1,0.8,0.7],
    # [2.4, -1.4, 0.000, 0.6,0.2,0.6,0.55],  # 3
    [2.0, -1.4, 0.000, 0.6,0.2,0.6,0.36],  

    # [1.5, -1.8, 0.000, 0.1,0.05,1.0,0.6],# 3U形弯 4
    #[2.24, -2, 3.082496, 0.2,0.1,1.2,0.8],  # 5
    # [2.74, -2, 3.082496, 0.1,0.1,1.1,0.7],#6 这里是15个点的
    [1.5, -1.8, 0.000, 1.0,0.1,0.6,0.7],# 3U形弯 4
    [2.2, -2, 3.082496, 0.1,0.1,0.8,0.65],  # 5
    [2.74, -2, 3.082496, 0.1,0.1,1.0,0.8],#6 这里是15个点的

    [3.15, -2.3, 3.082496, 1.0,0.1,0.8,0.9],#7 0.7 0.9
    #[3.15, -3.0, 3.082496, 1,0.1,0.6,0.4],#8
    [3.15, -3.1, 3.082496, 0.1,0.1,0.6,0.7],#9
    #[2.4,-2.8,0,0.1,0.1,0.4,0.3],
    #[2.1, -2.7, 3.082496,1.0,0.1,0.0,0.0],#10
    #[2.1, -2.7, 3.082496, 1,0.1,0.6,0.3],#10
    [0.6, -3.2, 3.082496, 1.0,0.1,0.8,0.8],
    [0, -2.8, 3.082496, 1.0,0.1,0.6,0.65],
    #11
    #12
]
goal_point=[
        #[0.1,-0.86,0,1.25],
        #[0.5,-0.9,0,1.25],
        [0.9,-0.9,0,1.25],#第二阶段第二个终点
    ]
goal_points=[
        [2.15 ,-2.64,0,3.00], #2.16 -2.63
    #[2.25,-3.0,0,2.0],#第一阶段终点
    #[1.70,-3.18,0,-0.6],#第一阶段终点
    [0.93,-1,0,1.7],#第二阶段第一个终点
    [0.5,-1,0,1.7],#第二阶段第二个终点
    [0.07,-0.9,0,1.57]#第二阶段第三个终点
]
key_points_bata=[
    [0.5,-1.8]
]
PASS_THRES_RADIUS = 0.3
spe_point_index=11 #15 jie chu dao er wei ma de dian

def is_passed(now_pos, next_waypoint,PASS_THRES_RADIUS_=0.15):
    dis_to_next_point = math.sqrt(
        (now_pos[0]-next_waypoint[0])**2+(now_pos[1]-next_waypoint[1])**2)
    #print('--dis:%.2f '%(dis_to_next_point))
    if dis_to_next_point <= PASS_THRES_RADIUS_:
        return True
    else:
        return False
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
def get_peaple_spe(msg):#人物识别回调函数
        long_hair_data=msg.long_hair
        meganei_data=msg.meganei
        print("long_hair_data:",long_hair_data,"meganei_data:",meganei_data)

class SpeedGover:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.goal_x=255
        self.goal_y=255
        self.odom_sub = 0
        self.amcl_sub=0

    def goalCB(self, msg):
        self.goal_x=msg.pose.position.x
        self.goal_y=msg.pose.position.y

        # # self.x = odom_data.pose.pose.position.x
        # # self.y = odom_data.pose.pose.position.y
        # new_pose=PoseStamped()
        # r = PoseStamped()
        # r.header.stamp = odom_data.header.stamp
        # r.header.frame_id = "base_link"
        # r.pose=odom_data.pose.pose
        # br = tf.TransformListener()
        # new_pose=br.transformPose("odom",r)
        # self.x=new_pose.pose.position.x
        # self.y=new_pose.pose.position.y
        # print("x:%.2f y:%.2f"%(x,y))

    def read_deal_amcl(self,amcl_data):
        self.x=amcl_data.pose.pose.position.x
        self.y=amcl_data.pose.pose.position.y

if __name__ == '__main__':
    rospy.init_node('nav_keypoints')
    listener = tf.TransformListener()
    # move_base_client = actionlib.SimpleActionClient(
    #     'move_base', MoveBaseAction)
    # move_base_client.wait_for_server()
    # print('-- move_base client has been started.')

    speed_gover = SpeedGover()
    listener.waitForTransform('/map','/base_link',rospy.Time(0),rospy.Duration(6.0))
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    reach_data_pub = rospy.Publisher('/goal_reach_data', String, queue_size=1)
    goal_pub=rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=1)
    #peple_data = rospy.Subscriber("peaple_data_topic", peaple_spe, laset_control.get_peaple_spe)#获取人物识别模型数据
    #TODO:记得打开上面的接收回调函数
    # 获取里程计消息
    # odom_sub = rospy.Subscriber("odometry/filtered", Odometry, speed_gover.get_odom_data)
    #amcl_pose = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, speed_gover.read_deal_amcl)
    #goal = rospy.Subscriber('/move_base_simple/goal', Path, speed_gover.goalCB)
    # speed_gover.amcl_sub = ucar_tf_pose

    # 动态参数调整服务器
    reconfig_client = 0
    reconfig_client_pp = 0
    local_planner = rosparam.get_param('/move_base/base_local_planner')
    reconfig_client_pp= dynamic_reconfigure.client.Client('/Pure_Pursuit')
    reconfig_vel=dynamic_reconfigure.client.Client('/velocity_smoother')
    if local_planner == 'teb_local_planner/TebLocalPlannerROS':
        reconfig_client = dynamic_reconfigure.client.Client(
            '/move_base/TebLocalPlannerROS')
    elif local_planner == 'dwa_local_planner/DWAPlannerROS':
        reconfig_client = dynamic_reconfigure.client.Client(
            '/move_base/DWAPlannerROS')
    elif local_planner == 'eband_local_planner/EBandPlannerROS':
        reconfig_client = dynamic_reconfigure.client.Client(
            '/move_base/EBandPlannerROS')

    print("--dynamic_reconfigure started! Waiting for move_base goal...")
    #params = {'max_vel_x': 2.0, 'max_vel_theta': 8.0}
    # config = reconfig_client.update_configuration(params)
    len_key_points=len(key_points)
    i=0
    speed_gover.goal_x=goal_point[0][0]#TODO:比赛时候记得修改终点坐标
    speed_gover.goal_y=goal_point[0][1]
    r = rospy.Rate(8)
    #goal_data = rospy.wait_for_message("/move_base_simple/goal", PoseStamped, timeout=None)
    #speed_gover.goal_x=goal_data.pose.position.x
    # speed_gover.goal_y=goal_data.pose.position.y
    #print(goal_data.pose.position.x,goal_data.pose.position.y)
    flag_mic=0
    flag_pub=0
    while not rospy.is_shutdown():
        (trans,rot) = listener.lookupTransform('/map','/base_link',rospy.Time(0))
        speed_gover.x=trans[0]
        speed_gover.y=trans[1]
        #goal_data = rospy.wait_for_message("/move_base_simple/goal", PoseStamped, timeout=None)
        # speed_gover.goal_x=goal_data.pose.position.x
        # speed_gover.goal_y=goal_data.pose.position.y
        # print(goal_data.pose.position.x,goal_data.pose.position.y)
        if  i<len_key_points and is_passed((speed_gover.x, speed_gover.y), (key_points[i][0], key_points[i][1])):
            params_pp={'Vcmd': key_points[i][5],'steering_gain':key_points[i][6]}
            params = {'max_vel_x': key_points[i][3],'acc_lim_theta':key_points[i][4]}
            print('max_vel_x:',key_points[i][3],'acc_lim_theta:',key_points[i][4])
            #config = reconfig_client.update_configuration(params)
            config_pp=reconfig_client_pp.update_configuration(params_pp)
            i=i+1
            print("--passed point [%d]" % (i))
            
            # if i==13:
            #     reconfig_vel_data={'teb_flage':False}
            #     reconfig_teb_data={'max_global_plan_lookahead_dist':0.3}
            #     config_vel=reconfig_vel.update_configuration(reconfig_vel_data)
            #     config_teb = reconfig_client.update_configuration(reconfig_teb_data)
            #     print("change decel!")
            # if i==spe_point_index:#在该关键点处重新规划终点 
            #     #goal_data = rospy.wait_for_message("/move_base_simple/goal", PoseStamped, timeout=None)
            #     speed_gover.goal_x=goal_point[2][0]
            #     speed_gover.goal_y=goal_point[2][1]
            #     print(goal_data.pose.position.x,goal_data.pose.position.y)
            #     print("we get goal!")
        # if  is_passed((speed_gover.x,speed_gover.y),(key_points_bata[0][0],key_points_bata[0][1]),0.3) and flag_pub==0:#经过关键点发布终点位置
        #     goal=goal_pose_stp(goal_points[1])#TODO 前往终点,记得与之前的终点值同时修改
        #     flag_pub=0
        #     for j in range(5):
        #         goal_pub.publish(goal)
            #if i==12:
             #   reconfig_vel={'decel_factor':5}
        if  is_passed((speed_gover.x, speed_gover.y), (speed_gover.goal_x, speed_gover.goal_y),0.23):
            twist=Twist()
            twist.angular.z = 0
            twist.linear.x = 0
            twist.linear.y = 0
            vel_pub.publish(twist)            
            reach_data="goal reach"
            params_pp_last={'Vcmd': 0,'steering_gain':0}
            params = {'max_vel_x': 0,'acc_lim_theta':0}
            config_pp=reconfig_client_pp.update_configuration(params_pp_last)
            config = reconfig_client.update_configuration(params)
            reach_data_pub.publish(reach_data)
            print("reach goal point")
            #while True:
            #r.sleep()
            # params_pp={'Vcmd': 0,'steering_gain':0}
            # params = {'max_vel_x':0,'max_vel_theta':0}
            # config = reconfig_client.update_configuration(params)
            # config_pp=reconfig_client_pp.update_configuration(params_pp)
            #vel_pub.publish(twist)
            #print("already get all points and at goal!")
            #exit(0)
                # if KeyboardInterrupt:
                #     print('操作已取消')
                #     exit(0)
            # 经过关键点后更新速度限制
        #rospy.spinonce()
    r.sleep()
        # for i in range(len(key_points)):
        #     # if i == 0:  # 第一段
        #     #     params = {'max_vel_x': 1.5, 'max_vel_theta': 2.0}
        #     #     config = recon_client.update_configuration(params)
        #     # elif i == len(key_points)-2:
        #     #     params = {'max_vel_x': 1.0, 'max_vel_theta': 2.0}
        #     #     config = recon_client.update_configuration(params)
        #     # elif i == len(key_points)-1:  # 目标点是最后一个
        #     #     params = {'max_vel_x': 0.6, 'max_vel_theta': 2.0}
        #     #     config = recon_client.update_configuration(params)
        #     # else:  # 多弯段
        #     #     params = {'max_vel_x': 0.8, 'max_vel_theta': 3.0}
        #     #     config = recon_client.update_configuration(params)
        #     move_goal = MoveBaseGoal()
        #     move_goal.target_pose.header.frame_id = 'map'
        #     move_goal.target_pose.pose.position.x = key_points[i][0]
        #     move_goal.target_pose.pose.position.y = key_points[i][1]
        #     [x, y, z, w] = tf.transformations.quaternion_from_euler(
        #         0, 0, key_points[i][2])
        #     move_goal.target_pose.pose.orientation.x = x
        #     move_goal.target_pose.pose.orientation.y = y
        #     move_goal.target_pose.pose.orientation.z = z
        #     move_goal.target_pose.pose.orientation.w = w
        #     move_base_client.send_goal(move_goal)
        #     print('-- move_base_goal: x:%.2f y:%.2f yaw:%.2f' %
        #           (key_points[i][0], key_points[i][1], key_points[i][2]))
        #     # 没接近下一个目标点就死循环
        #     if i < len(key_points)-1:
        #         while not is_passed(get_model_state, (key_points[i][0], key_points[i][1])):
        #             pass
        #     else:  # 最后一个点采用move_base service判断到达
        #         move_base_client.wait_for_result()
        #         break
        #     print('-- reached a goal.')

        # rospy.spin()
