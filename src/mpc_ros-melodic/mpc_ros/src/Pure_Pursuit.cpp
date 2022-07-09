/*
# Copyright 2018 HyphaROS Workshop.
# Latest Modifier: HaoChih, LIN (hypha.ros@gmail.com)
# Original Author: ChanYuan KUO & YoRu LU
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
*/

#include <iostream>
#include <map>
#include <math.h>
#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>
#include "pid.h"
#include <Eigen/Core>
#include <Eigen/QR>
#include <vector>

#include <string>
#include <fstream>

#include <dynamic_reconfigure/server.h>
#include <mpc_ros/paramsConfig.h>
using namespace std;
using std::string;

/********************/
/* CLASS DEFINITION */
/********************/
class PurePursuit
{
    public:
        PurePursuit();
        void initMarker();
        bool isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose);
        bool isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos);
        double getYawFromPose(const geometry_msgs::Pose& carPose);        
        void DynamicCB(mpc_ros::paramsConfig &config, uint32_t level);
        double polyeval(Eigen::VectorXd coeffs, double x);        
        Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);
        
        double getCar2GoalDist();

	double getW(double _alpha);	
		
        double get_alpha(const geometry_msgs::Pose& carPose);

    private:
        

        ros::NodeHandle n_;
        ros::Subscriber odom_sub, path_sub, goal_sub, amcl_sub;
        ros::Publisher ackermann_pub, cmdvel_pub, marker_pub;
        ros::Timer timer1, timer2;
        tf::TransformListener tf_listener;

        ros::Time tracking_stime;
        ros::Time tracking_etime;
        ros::Time tracking_time;
        int tracking_time_sec;
        int tracking_time_nsec;
        
        //time flag
        bool start_timef = false;
        bool end_timef = false;

        std::ofstream file;
    
        double _waypointsDist = -1.0;
        double _pathLength = 8.0;
        bool _path_computed = false;
        //pid
        PID_t _pid_x, _pid_w;
        string _map_frame, _odom_frame, _car_frame,_cmd_topic;
       
        unsigned int idx = 0;

        visualization_msgs::Marker points, line_strip, goal_circle;
        geometry_msgs::Point odom_goal_pos, goal_pos;
        geometry_msgs::Twist cmd_vel;
        ackermann_msgs::AckermannDriveStamped ackermann_cmd;
        nav_msgs::Odometry odom;
        nav_msgs::Path map_path, odom_path, odom_path_w, _odom_path;
        nav_msgs::Odometry spe_odom;
        geometry_msgs::Point spe_point_1,spe_point_2;

        dynamic_reconfigure::Server<mpc_ros::paramsConfig> *   dynamic_reconfigure_server;
        dynamic_reconfigure::Server<mpc_ros::paramsConfig>::CallbackType dynamic_reconfigure_callback;

        int _downSampling;
        double pid_x_kp,pid_x_ki,pid_x_kd,pid_x_integral_limit,pid_x_output_limit;
        double pid_w_kp,pid_w_ki,pid_w_kd,pid_w_integral_limit,pid_w_output_limit;
        double _control_lastsecs;
        double L, Lfw, Vcmd, lfw, steering, velocity;
        double V_param_0,W_param_0,W_param_1,V_param_1,W_param_2,V_param_2,W_param_3,V_param_3;
        double set_w_1,set_w_2,set_w_3;
        double w;
        double steering_gain, max_w, base_angle, goal_radius, speed_incremental;
        int controller_freq;
        bool foundForwardPt, goal_received, goal_reached, cmd_vel_mode, debug_mode, smooth_accel,pid_mode;

        void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
        void amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg);
        void controlLoopCB(const ros::TimerEvent&);

}; // end of class


PurePursuit::PurePursuit()
{
    //Private parameters handler
    ros::NodeHandle pn("~");

    //Car parameter
    pn.param("L", L, 0.26); // length of car
    pn.param("Vcmd", Vcmd, 1.0);// reference speed (m/s)
    pn.param("Lfw", Lfw, 0.5); // forward look ahead distance (m) 前瞻距离
    pn.param("lfw", lfw, 0.13); // distance between front the center of car 车前到车重心距离
    pn.param("set_w_1", set_w_1, 0.1);//直道阙值
    pn.param("set_w_2", set_w_2, 0.1);//小弯阙值
    pn.param("set_w_3", set_w_3, 0.1);//中大弯阙值
    pn.param("V_param_0", V_param_0, 1.0);
    pn.param("W_param_0", W_param_0, 1.0);//直到参数
    pn.param("V_param_1", V_param_1, 1.0);
    pn.param("W_param_1", W_param_1, 1.0);//小湾参数
    pn.param("V_param_1", V_param_2, 1.0);
    pn.param("W_param_1", W_param_2, 1.0);//中弯参数
    pn.param("V_param_1", V_param_3, 1.0);
    pn.param("W_param_1", W_param_3, 1.0);//大弯参数
    //pid_x
    pn.param("pid_x/kp", pid_x_kp, 1.0);
    pn.param("pid_x/ki", pid_x_ki, 0.01);
    pn.param("pid_x/kd", pid_x_kd, 0.55);
    pn.param("pid_x_integral_limit", pid_x_integral_limit, 1.0);
    pn.param("pid_x_output_limit", pid_x_output_limit, 1.0);
    //pid_w
    pn.param("pid_w/kp", pid_w_kp, 1.0);
    pn.param("pid_w/ki",pid_w_ki, 0.01);
    pn.param("pid_w/kd",pid_w_kd, 0.55);
    pn.param("pid_w_integral_limit", pid_w_integral_limit, 1.0);
    pn.param("pid_w_output_limit", pid_w_output_limit, 1.0);
    //Controller parameter
    pn.param("controller_freq", controller_freq, 20);
    pn.param("steering_gain", steering_gain, 1.0);
    pn.param("max_w", max_w, 1.0);
    pn.param("goal_radius", goal_radius, 0.5); // goal radius (m) 为在给定的转向角下后轴遵循着的圆的半径
    pn.param("base_angle", base_angle, 0.0); // neutral point of servo (rad) 
    pn.param("cmd_vel_mode", cmd_vel_mode, false); // whether or not publishing cmd_vel
    pn.param("debug_mode", debug_mode, false); // debug mode
    pn.param("smooth_accel", smooth_accel, true); // smooth the acceleration of car
    pn.param("speed_incremental", speed_incremental, 0.5); // speed incremental value (discrete acceleraton), unit: m/s 速度增量值，离散加速度
    pn.param("pid_mode", pid_mode, false); 
    //Parameter for topics & Frame name
    pn.param<std::string>("map_frame", _map_frame, "map" ); //*****for mpc, "odom"
    pn.param<std::string>("odom_frame", _odom_frame, "odom");
    pn.param<std::string>("car_frame", _car_frame, "base_link" );
    pn.param<std::string>("cmd_topic", _cmd_topic, "cmd_vel" );
    //Publishers and Subscribers
    odom_sub = n_.subscribe("/odometry/filtered", 1, &PurePursuit::odomCB, this);
    //path_sub = n_.subscribe("/pure_pursuit/global_planner", 1, &PurePursuit::pathCB, this);
    //goal_sub = n_.subscribe("/pure_pursuit/goal", 1, &PurePursuit::goalCB, this);/move_base/GlobalPlanner/plan
    path_sub = n_.subscribe("/move_base/TebLocalPlannerROS/global_plan", 1, &PurePursuit::pathCB, this);
    //  path_sub = n_.subscribe("/move_base/GlobalPlanner/plan", 1, &PurePursuit::pathCB, this);
    goal_sub = n_.subscribe("/move_base_simple/goal", 1, &PurePursuit::goalCB, this); //目标点
    amcl_sub = n_.subscribe("/amcl_pose", 5, &PurePursuit::amclCB, this);
    marker_pub = n_.advertise<visualization_msgs::Marker>("/pure_pursuit/path_marker", 10);
    ackermann_pub = n_.advertise<ackermann_msgs::AckermannDriveStamped>("/pure_pursuit/ackermann_cmd", 1);
    if(cmd_vel_mode) cmdvel_pub = n_.advertise<geometry_msgs::Twist>(_cmd_topic, 1);    

    //Timer
    timer1 = n_.createTimer(ros::Duration((1.0)/controller_freq), &PurePursuit::controlLoopCB, this); // Duration(0.05) -> 20Hz

    spe_point_1.x=2.0;
    spe_point_1.y=-1.4;
    spe_point_2.x=2.0;
    spe_point_2.y=-1.95;
    //Init variables
    foundForwardPt = false;
    goal_received = false;
    goal_reached = false;
    velocity = 0.0;
    w = 0.0;
    steering = base_angle;

    idx = 0;
    file.open("/home/ucar/ucar_ws/src/ucar_controller/log_info/pure_pursuit_bata.csv");

    //Dynamic reconfige init

  dynamic_reconfigure_callback = boost::bind(&PurePursuit::DynamicCB, this, _1, _2);

  dynamic_reconfigure_server = new dynamic_reconfigure::Server<mpc_ros::paramsConfig>(pn);
  dynamic_reconfigure_server->setCallback(dynamic_reconfigure_callback);
    // f = boost::bind(&PurePursuit::DynamicCB,_1,_2);
    // server.setCallback(f);

    //Show info
    ROS_INFO("[param] base_angle: %f", base_angle);
    ROS_INFO("[param] Vcmd: %f", Vcmd);
    ROS_INFO("[param] Lfw: %f", Lfw);

    //Visualization Marker Settings
    initMarker();

    cmd_vel = geometry_msgs::Twist();
    ackermann_cmd = ackermann_msgs::AckermannDriveStamped();
	//pid init
    pid_init(&_pid_x, PID_MODE_DERIVATIV_CALC, 0.001f);
    pid_set_parameters(&_pid_x, pid_x_kp, pid_x_ki, pid_x_kd, pid_x_integral_limit, pid_x_output_limit);
    pid_init(&_pid_w, PID_MODE_DERIVATIV_CALC, 0.001f);
    pid_set_parameters(&_pid_w, pid_w_kp, pid_w_ki, pid_w_kd, pid_w_integral_limit, pid_w_output_limit);
}
//动态调参回调函数
void PurePursuit::DynamicCB(mpc_ros::paramsConfig &config, uint32_t level){
    Vcmd=config.Vcmd;
    Lfw=config.Lfw;
    steering_gain=config.steering_gain;
    base_angle=config.base_angle;

}
void PurePursuit::initMarker()
{
    points.header.frame_id = line_strip.header.frame_id = goal_circle.header.frame_id = "odom";
    points.ns = line_strip.ns = goal_circle.ns = "Markers";
    points.action = line_strip.action = goal_circle.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = goal_circle.pose.orientation.w = 1.0;
    points.id = 0;
    line_strip.id = 1;
    goal_circle.id = 2;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    goal_circle.type = visualization_msgs::Marker::CYLINDER;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    //LINE_STRIP markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;

    goal_circle.scale.x = goal_radius;
    goal_circle.scale.y = goal_radius;
    goal_circle.scale.z = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    //goal_circle is yellow
    goal_circle.color.r = 1.0;
    goal_circle.color.g = 1.0;
    goal_circle.color.b = 0.0;
    goal_circle.color.a = 0.5;
}


void PurePursuit::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    this->odom = *odomMsg;
}


//路径回调函数
void PurePursuit::pathCB(const nav_msgs::Path::ConstPtr& pathMsg) //jaewan
{
    this->map_path = *pathMsg;

    if(goal_received && !goal_reached)
    {    
        nav_msgs::Path odom_path = nav_msgs::Path();
        try
        {
            double total_length = 0.0;
            int sampling = _downSampling;

            //find waypoints distance
            if(_waypointsDist <=0.0)
            {        
                double dx = pathMsg->poses[1].pose.position.x - pathMsg->poses[0].pose.position.x;
                double dy = pathMsg->poses[1].pose.position.y - pathMsg->poses[0].pose.position.y;
                _waypointsDist = sqrt(dx*dx + dy*dy);
                _downSampling = int(_pathLength/10.0/_waypointsDist);
            }            

            // Cut and downsampling the path
            for(int i =0; i< pathMsg->poses.size(); i++)
            {
                if(total_length > _pathLength)
                    break;

                if(sampling == _downSampling)
                {   
                    geometry_msgs::PoseStamped tempPose;
                    tf_listener.transformPose(_odom_frame, ros::Time(0) , pathMsg->poses[i], _map_frame, tempPose);                     
                    odom_path.poses.push_back(tempPose);  
                    sampling = 0;
                }
                total_length = total_length + _waypointsDist; 
                sampling = sampling + 1;  
            }
           
            if(odom_path.poses.size() >= 6 )
            {
                _odom_path = odom_path; // Path waypoints in odom frame
                _path_computed = true;
                // publish odom path
                odom_path.header.frame_id = _odom_frame;
                odom_path.header.stamp = ros::Time::now();
            }
            else
            {
                //  cout << odom_path.poses.size() << endl;
                // cout << "Failed to path generation" << endl;
                _waypointsDist = -1;
            }
            //DEBUG            
            //cout << endl << "N: " << odom_path.poses.size() << endl 
            //<<  "Car path[0]: " << odom_path.poses[0];
            // << ", path[N]: " << _odom_path.poses[_odom_path.poses.size()-1] << endl;
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }
    
}

//终点回调函数
void PurePursuit::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    this->goal_pos = goalMsg->pose.position;    
    try
    {
        geometry_msgs::PoseStamped odom_goal;
        tf_listener.transformPose("odom", ros::Time(0) , *goalMsg, "map" ,odom_goal);
        odom_goal_pos = odom_goal.pose.position;
        goal_received = true;
        goal_reached = false;

        /*Draw Goal on RVIZ*/
        goal_circle.pose = odom_goal.pose;
        marker_pub.publish(goal_circle);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}
//四元数变换成偏航角
double PurePursuit::getYawFromPose(const geometry_msgs::Pose& carPose)
{
    float x = carPose.orientation.x;
    float y = carPose.orientation.y;
    float z = carPose.orientation.z;
    float w = carPose.orientation.w;

    double tmp,yaw;
    tf::Quaternion q(x,y,z,w);
    tf::Matrix3x3 quaternion(q);
    quaternion.getRPY(tmp,tmp, yaw);

    return yaw;
}
//判断是否为前方点
bool PurePursuit::isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose)
{
    float car2wayPt_x = wayPt.x - carPose.position.x;
    float car2wayPt_y = wayPt.y - carPose.position.y;
    double car_theta = getYawFromPose(carPose);

    float car_car2wayPt_x = cos(car_theta)*car2wayPt_x + sin(car_theta)*car2wayPt_y;
    float car_car2wayPt_y = -sin(car_theta)*car2wayPt_x + cos(car_theta)*car2wayPt_y;

    if(car_car2wayPt_x >0) /*is Forward WayPt*/
        return true;
    else
        return false;
}

//判断一个点是否符合做为下一个目标点
bool PurePursuit::isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos)
{
    double dx = wayPt.x - car_pos.x;
    double dy = wayPt.y - car_pos.y;
    double dist = sqrt(dx*dx + dy*dy);

    if(dist < Lfw)
        return false;
    else if(dist >= Lfw)
        return true;
}
//计算α
double PurePursuit::get_alpha(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point carPose_pos = carPose.position;
    double carPose_yaw = getYawFromPose(carPose);
    geometry_msgs::Point odom_path_wayPt;
    geometry_msgs::Point forwardPt;
    geometry_msgs::Point odom_car2WayPtVec;
    foundForwardPt = false;

    if(!goal_reached){

        for(int i =0; i< _odom_path.poses.size(); i++)
        {
            geometry_msgs::PoseStamped map_path_pose = _odom_path.poses[i];
            geometry_msgs::PoseStamped odom_path_pose;

            try
            {
                tf_listener.transformPose("odom", ros::Time(0) , map_path_pose, "map" ,odom_path_pose);
                odom_path_wayPt = odom_path_pose.pose.position;
                bool _isForwardWayPt = isForwardWayPt(odom_path_wayPt,carPose);

                if(_isForwardWayPt)
                {
                    bool _isWayPtAwayFromLfwDist = isWayPtAwayFromLfwDist(odom_path_wayPt,carPose_pos);
                    if(_isWayPtAwayFromLfwDist)
                    {
                        forwardPt = odom_path_wayPt;
                        foundForwardPt = true;
                        break;
                    }
                }
            }
            catch(tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        }
        
    }
    else if(goal_reached)
    {
        forwardPt = odom_goal_pos;
        foundForwardPt = false;
        //ROS_INFO("goal REACHED!");_control_lastsecs
    }

    /*Visualized Target Point on RVIZ*/
    /*Clear former target point Marker*/
    points.points.clear();
    line_strip.points.clear();
    
    if(foundForwardPt && !goal_reached)
    {
        points.points.push_back(carPose_pos);
        points.points.push_back(odom_path_wayPt);
        line_strip.points.push_back(carPose_pos);
        line_strip.points.push_back(odom_path_wayPt);
    }

    marker_pub.publish(points);
    marker_pub.publish(line_strip);
 
    double alpha_t = atan2(odom_path_wayPt.y - carPose_pos.y,odom_path_wayPt.x - carPose_pos.x) - carPose_yaw;
	
    return alpha_t;
}
//计算当前点距离目标点的距离
double PurePursuit::getCar2GoalDist()
{
    geometry_msgs::Point car_pose = this->odom.pose.pose.position;
    double car2goal_x = this->odom_goal_pos.x - car_pose.x;
    double car2goal_y = this->odom_goal_pos.y - car_pose.y;

    return sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);
}

double PurePursuit::getW(double _alpha)
{
    return 2*sin(_alpha)/Lfw;//计算圆弧曲率
}

double PurePursuit::polyeval(Eigen::VectorXd coeffs, double x) 
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) 
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}
//QR分解V_param_2
Eigen::VectorXd PurePursuit::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) 
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
        A(i, 0) = 1.0;

    for (int j = 0; j < xvals.size(); j++) 
    {
        for (int i = 0; i < order; i++) 
            A(j, i + 1) = A(j, i) * xvals(j);
    }

    auto Q = A.householderQr();//正交分析
    auto result = Q.solve(yvals);
    return result;
}
void PurePursuit::amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg)
{

    if(this->goal_received)
    {
        double car2goal_x = this->goal_pos.x - amclMsg->pose.pose.position.x;
        double car2goal_y = this->goal_pos.y - amclMsg->pose.pose.position.y;
        double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);
        if(dist2goal < this->goal_radius)
        {
            if(start_timef)
            {
                tracking_etime = ros::Time::now();
                tracking_time_sec = tracking_etime.sec - tracking_stime.sec; 
                tracking_time_nsec = tracking_etime.nsec - tracking_stime.nsec; 
                
                file << "tracking time"<< "," << tracking_time_sec << "," <<  tracking_time_nsec << "\n";

                file.close();

                start_timef = false;
            }
            this->goal_reached = true;
            this->goal_received = false;
            this->_path_computed = false;

            ROS_INFO("Goal Reached !");
            cout << "tracking time: " << tracking_time_sec << "." << tracking_time_nsec << endl;

        }
    }
    // if(reache_sep_point){//如果到达转弯点

    // }
}


void PurePursuit::controlLoopCB(const ros::TimerEvent&)
{

    geometry_msgs::Pose carPose = this->odom.pose.pose;
    geometry_msgs::Twist carVel = this->odom.twist.twist;

    tf::Pose pose;
    tf::poseMsgToTF(carPose, pose);
    double alpha = tf::getYaw(pose.getRotation());


    if(this->goal_received && !this->goal_reached && this->_path_computed)
    {
        if(!start_timef)
        {
            tracking_stime == ros::Time::now();


            start_timef = true;
        }

        double alpha = get_alpha(carPose);  
        
	    //this->w = this->base_angle + getW(alpha)*this->steering_gain;
        // if(this->w >= this->set_w){
        // this->
        // }
        if(this->w > max_w)
        {
            this->w = max_w;
        }
        else if(this->w < -max_w)
        {
            this->w = -max_w;
        }

        nav_msgs::Odometry odom_w = odom; 
        nav_msgs::Path odom_path_w = _odom_path;   


        // Update system states: X=[x, y, theta, v]
        const double px = odom_w.pose.pose.position.x; //pose: odom frame
        const double py = odom_w.pose.pose.position.y;
        tf::Pose pose;
        tf::poseMsgToTF(odom_w.pose.pose, pose);
        const double theta = tf::getYaw(pose.getRotation());//转换四元数到偏航角

        // Waypoints related parameters 航路点相关参数
        const int N = odom_path_w.poses.size(); // Number of waypoints
        const double costheta = cos(theta);
        const double sintheta = sin(theta);

        // Convert to the vehicle coordinate system转换为车辆坐标系
        Eigen::VectorXd x_veh(N);
        Eigen::VectorXd y_veh(N);
        for(int i = 0; i < N; i++) 
        {
            const double dx = odom_path_w.poses[i].pose.position.x - px;
            const double dy = odom_path_w.poses[i].pose.position.y - py;
            x_veh[i] = dx * costheta + dy * sintheta;
            y_veh[i] = dy * costheta - dx * sintheta;
        }
          
        // Fit waypoints
        auto coeffs = polyfit(x_veh, y_veh, 3); 

        const double cte  = polyeval(coeffs, 0.0);//coeffs所有元素累加
        const double etheta = atan(coeffs[1]);
        // cout << "alpha: " << alpha << endl;
        // cout << "cte: " << cte << endl;
        // cout << "etheta: " << etheta << endl;
        // cout << "v : " << cmd_vel.linear.x  << endl;
        // cout << "w : " << cmd_vel.angular.z  << endl;
        
        idx++;
        file << idx<< "cte:" << cte << ",etheta:" <<  etheta << ",cmd_vel.linear.x:" << cmd_vel.linear.x << ",cmd_vel.angular.z:" << cmd_vel.angular.z << "\n";
        
            /*Estimate Gas Input*/
        if(!this->goal_reached)
        {
            if(this->smooth_accel) 
                this->velocity = std::min(this->velocity + this->speed_incremental, this->Vcmd);
/*
<set_w_1
set_w_1~set_w_2
set_w_2~set_w_3
>set_w_3
*/
            else
            { 
                double secs = ros::Time::now().toSec();
                double dt = secs - _control_lastsecs;
                 _control_lastsecs = secs;
                 if(pid_mode){
                this->Vcmd=pid_calculate(&_pid_x, abs(etheta), 0, 0, dt);
                 this->velocity = this->Vcmd;
                 }
                 else{
                     this->velocity = this->Vcmd;
                 }
                //this->w=pid_calculate(&_pid_w, etheta, 0, 0, dt);
                //this->velocity = this->Vcmd;
               // this->w =( this->base_angle + getW(alpha)*this->steering_gain);
                 if(abs(etheta) >= this->set_w_1)//直道路 <set_w_1
                {
                    this->velocity = this->Vcmd*this->V_param_0;
                    this->w =( this->base_angle + getW(alpha)*this->steering_gain)*this->W_param_0;
                }
                else{
                    this->velocity = this->Vcmd;
                    this->w =( this->base_angle + getW(alpha)*this->steering_gain);
                }
                        // }
                if(this->w > max_w)
                {
                    this->w = max_w;
                }
                else if(this->w < -max_w)
                {
                    this->w = -max_w;
                }

            //    double px_point =abs(odom_w.pose.pose.x-spe_point_1.x);
            //    double py_point =abs(odom_w.pose.pose.y-spe_point_1.y);
            //    if(px_point<=0.05&&py_point<=0.05){
            //        spe_flag=true;
            //         this->velocity = this->Vcmd*this->V_param_1;
            //         this->w =( this->base_angle + getW(alpha)*this->steering_gain)*this->W_param_1;
            //    }

            //    else if(abs(etheta) >= this->set_w_1 && abs(etheta) < this->set_w_2){//小弧度弯 set_w_1~set_w_2
            //         this->velocity =this->Vcmd*this->V_param_1;
            //         this->w = (this->base_angle + getW(alpha)*this->steering_gain)*this->W_param_1;
            //     }
            //     else if(abs(etheta) >= this->set_w_2 && abs(etheta) < this->set_w_3){//中弧度弯 set_w_2~set_w_3
            //     this->velocity =this->Vcmd*this->V_param_2;
            //     this->w = (this->base_angle + getW(alpha)*this->steering_gain)*this->W_param_2;
            //     }
            //      else if(abs(etheta) >= this->set_w_3){//大弧度弯>set_w_3
            //         this->velocity = this->Vcmd*this->V_param_3;
            //         this->w =( this->base_angle + getW(alpha)*this->steering_gain)*this->W_param_3;
            //     }
            //     }
            if(debug_mode) ROS_INFO("Velocity = %.2f, w = %.2f", this->velocity, this->w);
            
        }
        
    }

    else
    {
        this->velocity = 0.0;
		this->w = 0;
    }
    

    if(this->cmd_vel_mode)
    {
        this->cmd_vel.linear.x = this->velocity;
        this->cmd_vel.angular.z = this->w;
	    this->cmdvel_pub.publish(this->cmd_vel);
    }   
}

}
/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "PurePursuit");
    PurePursuit controller;
    ros::AsyncSpinner spinner(2); // Use multi threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
