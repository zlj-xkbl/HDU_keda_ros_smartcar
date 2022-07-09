#include <csignal>
#include <iostream>

// ROS
#include "ros/ros.h"

// ROS sensor messages
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

// ROS image geometry
#include <image_geometry/pinhole_camera_model.h>

// ROS transform
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

// ROS CvBridge
#include "cv_bridge/cv_bridge.h"

// OpenCV
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/cudaimgproc.hpp>

using namespace std;
using namespace sensor_msgs;
using namespace cv;

// Define global variables
bool camera_model_computed = false;
bool show_detections;
float marker_size;
image_geometry::PinholeCameraModel camera_model;
Mat distortion_coefficients;
Matx33d intrinsic_matrix;
Ptr<aruco::DetectorParameters> detector_params;
Ptr<cv::aruco::Dictionary> dictionary;
string marker_tf_prefix;
string output_topic;
int blur_window_size = 7;
bool enable_blur = true;
ros::Publisher pub;

void int_handler(int x) {
    // disconnect and exit gracefully
    if (show_detections) {
        cv::destroyAllWindows();
    }
    ros::shutdown();
    exit(0);
}

tf2::Vector3 cv_vector3d_to_tf_vector3(const Vec3d &vec) {
    return {vec[0], vec[1], vec[2]};
}

tf2::Quaternion cv_vector3d_to_tf_quaternion(const Vec3d &rotation_vector) {
    Mat rotation_matrix;
    auto ax = rotation_vector[0], ay = rotation_vector[1], az = rotation_vector[2];
    auto angle = sqrt(ax * ax + ay * ay + az * az);
    auto cosa = cos(angle * 0.5);
    auto sina = sin(angle * 0.5);
    auto qx = ax * sina / angle;
    auto qy = ay * sina / angle;
    auto qz = az * sina / angle;
    auto qw = cosa;
    tf2::Quaternion q;
    q.setValue(qx, qy, qz, qw);
    return q;
}

tf2::Transform create_transform(const Vec3d &tvec, const Vec3d &rotation_vector) {
    tf2::Transform transform;
    transform.setOrigin(cv_vector3d_to_tf_vector3(tvec));
    transform.setRotation(cv_vector3d_to_tf_quaternion(rotation_vector));
    return transform;
}

void callback_camera_info(const CameraInfoConstPtr &msg) {
    if (camera_model_computed) {
        return;
    }
    camera_model.fromCameraInfo(msg);
    camera_model.distortionCoeffs().copyTo(distortion_coefficients);
    intrinsic_matrix = camera_model.intrinsicMatrix();
    camera_model_computed = true;
    ROS_INFO("camera model is computed");
}

void callback(const ImageConstPtr &image_msg) {
    if (!camera_model_computed) {
        ROS_INFO("camera model is not computed yet");
        return;
    }

    string frame_id = image_msg->header.frame_id;
    auto image = cv_bridge::toCvShare(image_msg)->image;
    
    
    cv::flip(image,image,1);
    //cv::cuda::GpuMat imagegpu(image);
    //cv::cvtColor(image, image, cv::COLOR_RGB2BGR);  // TODO: deleteme!!!
   // cv::threshold(image,image,100,255,cv::THRESH_BINARY);
    //cv::Mat display_image(image);
    // Smooth the image to improve detection results
    if (enable_blur) {
        GaussianBlur(image, image, Size(blur_window_size, blur_window_size), 0,
                     0);
    }

    // Detect the markers
    vector<int> ids;
    vector<vector<Point2f>> corners, rejected;
    aruco::detectMarkers(image, dictionary, corners, ids, detector_params, rejected);

    // // Show image if no markers are detected
    // if (ids.empty()) {
    //     //ROS_INFO("Markers not found");
    //     if (show_detections) {
    //         //imshow("markers", display_image);
    //         auto key = waitKey(1);
    //         if (key == 27) {
    //             ROS_INFO("ESC pressed, exit the program");
    //             ros::shutdown();
    //         }
    //     }
    //     return;
    // }

    // Compute poses of markers
    vector<Vec3d> rotation_vectors, translation_vectors;
    aruco::estimatePoseSingleMarkers(corners, marker_size, intrinsic_matrix, distortion_coefficients,
                                     rotation_vectors, translation_vectors);
    for (auto i = 0; i < rotation_vectors.size(); ++i) {
        aruco::drawAxis(image, intrinsic_matrix, distortion_coefficients,
                        rotation_vectors[i], translation_vectors[i], marker_size * 0.5f);
    }

    // // Draw marker poses
    // if (show_detections) {
    //     //aruco::drawDetectedMarkers(display_image, corners, ids);
    //     //imshow("markers", display_image);
    //     auto key = waitKey(1);
    //     if (key == 27) {
    //         ROS_INFO("ESC pressed, exit the program");
    //         ros::shutdown();
    //     }
    // }

    // Publish TFs for each of the markers
    static tf2_ros::TransformBroadcaster br;
    auto stamp = ros::Time::now();

    // Create and publish tf message for each marker
    for (auto i = 0; i < rotation_vectors.size(); ++i) {
        auto translation_vector = translation_vectors[i];
        auto rotation_vector = rotation_vectors[i];
        auto transform = create_transform(translation_vector, rotation_vector);
        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header.stamp = stamp;
        tf_msg.header.frame_id = frame_id;
        stringstream ss;
        ss << marker_tf_prefix << ids[i];
        tf_msg.child_frame_id = ss.str();
        tf_msg.transform.translation.x = transform.getOrigin().getX();
        tf_msg.transform.translation.y = transform.getOrigin().getY();
        tf_msg.transform.translation.z = transform.getOrigin().getZ();
        tf_msg.transform.rotation.x = transform.getRotation().getX();
        tf_msg.transform.rotation.y = transform.getRotation().getY();
        tf_msg.transform.rotation.z = transform.getRotation().getZ();
        tf_msg.transform.rotation.w = transform.getRotation().getW();
        br.sendTransform(tf_msg);
        pub.publish(tf_msg);
    }
}

int main(int argc, char **argv) {
    map<string, aruco::PREDEFINED_DICTIONARY_NAME> dictionary_names = {
            {"DICT_4X4_50",         aruco::DICT_4X4_50},
            {"DICT_4X4_100",        aruco::DICT_4X4_100},
            {"DICT_4X4_250",        aruco::DICT_4X4_250},
            {"DICT_4X4_1000",       aruco::DICT_4X4_1000},
            {"DICT_5X5_50",         aruco::DICT_5X5_50},
            {"DICT_5X5_100",        aruco::DICT_5X5_100},
            {"DICT_5X5_250",        aruco::DICT_5X5_250},
            {"DICT_5X5_1000",       aruco::DICT_5X5_1000},
            {"DICT_6X6_50",         aruco::DICT_6X6_50},
            {"DICT_6X6_100",        aruco::DICT_6X6_100},
            {"DICT_6X6_250",        aruco::DICT_6X6_250},
            {"DICT_6X6_1000",       aruco::DICT_6X6_1000},
            {"DICT_7X7_50",         aruco::DICT_7X7_50},
            {"DICT_7X7_100",        aruco::DICT_7X7_100},
            {"DICT_7X7_250",        aruco::DICT_7X7_250},
            {"DICT_7X7_1000",       aruco::DICT_7X7_1000},
            {"DICT_ARUCO_ORIGINAL", aruco::DICT_ARUCO_ORIGINAL}};
    signal(SIGINT, int_handler);

    // Initalize ROS node
    ros::init(argc, argv, "simple_aruco_detector");
    ros::NodeHandle nh("~");
    string rgb_topic, rgb_info_topic, dictionary_name;
    nh.param("camera", rgb_topic, string("/kinect2/hd/image_color_rect"));
    nh.param("camera_info", rgb_info_topic, string("/kinect2/hd/camera_info"));
    nh.param("show_detections", show_detections, true);
    nh.param("tf_prefix", marker_tf_prefix, string("marker"));
    nh.param("marker_size", marker_size, 0.09f);
    nh.param("enable_blur", enable_blur, true);
    nh.param("blur_window_size", blur_window_size, 7);
    detector_params = aruco::DetectorParameters::create();
    detector_params->cornerRefinementMethod =  aruco::CORNER_REFINE_SUBPIX;
    detector_params->minDistanceToBorder=5;
    detector_params->minMarkerPerimeterRate=0.2;
    detector_params->cornerRefinementMaxIterations= 15;
    nh.param("aruco_dictionary", dictionary_name, string("DICT_4X4_50"));
    nh.param("aruco_adaptiveThreshWinSizeStep", detector_params->adaptiveThreshWinSizeStep, 4);
    nh.param("output_topic", output_topic, string("transforms"));
    int queue_size = 1;

    // Configure ARUCO marker detector
    dictionary = aruco::getPredefinedDictionary(dictionary_names[dictionary_name]);

    if (show_detections) {
        namedWindow("markers", cv::WINDOW_KEEPRATIO);
    }
    ros::Subscriber rgb_sub = nh.subscribe(rgb_topic.c_str(), queue_size, callback);
    ros::Subscriber rgb_info_sub = nh.subscribe(rgb_info_topic.c_str(), queue_size, callback_camera_info);
    pub = nh.advertise<geometry_msgs::TransformStamped>(output_topic.c_str(), queue_size);

    ros::spin();
    return 0;
}
