#ifndef _COMMON_H_
#define _COMMON_H_

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdio.h>

#include <opencv2/core/core.hpp>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

// TF
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Geometry>

Eigen::Matrix<double, 3, 3> toMatrix3d(const cv::Mat& cvMat3)
{
    Eigen::Matrix<double, 3, 3> M;

    M << cvMat3.at<float>(0, 0), cvMat3.at<float>(0, 1), cvMat3.at<float>(0, 2),
        cvMat3.at<float>(1, 0), cvMat3.at<float>(1, 1), cvMat3.at<float>(1, 2),
        cvMat3.at<float>(2, 0), cvMat3.at<float>(2, 1), cvMat3.at<float>(2, 2);

    return M;
}

std::vector<float> toQuaternion(const cv::Mat& M)
{
    Eigen::Matrix<double, 3, 3> eigMat = toMatrix3d(M);
    Eigen::Quaterniond q(eigMat);

    std::vector<float> v(4);
    v[0] = q.x();
    v[1] = q.y();
    v[2] = q.z();
    v[3] = q.w();

    return v;
}

// Publish TF to ROS
void publish_tf(const cv::Mat& t_Tcw, const ros::Time& t_stamp,
    const std::string& t_frame_id, const std::string& t_child_frame_id)
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = t_stamp;
    transformStamped.header.frame_id = t_frame_id;
    transformStamped.child_frame_id = t_child_frame_id;

    cv::Mat Rcw = t_Tcw.rowRange(0, 3).colRange(0, 3);
    cv::Mat tcw = t_Tcw.rowRange(0, 3).col(3);
    cv::Mat Rwc = Rcw.t(); 
    cv::Mat twc = -Rwc * tcw;

    vector<float> q = toQuaternion(Rwc);

    transformStamped.transform.translation.x = twc.at<float>(0, 0);
    transformStamped.transform.translation.y = twc.at<float>(0, 1);
    transformStamped.transform.translation.z = twc.at<float>(0, 2);

    transformStamped.transform.rotation.x = q[0];
    transformStamped.transform.rotation.y = q[1];
    transformStamped.transform.rotation.z = q[2];
    transformStamped.transform.rotation.w = q[3];

    // transformStamped.transform.translation()
    // transformStamped.transform.rotation(q[0], q[1], q[2], q[3]);

    br.sendTransform(transformStamped);
}

void signal_handler_callback(int sig)
{
    ROS_INFO("Ctrl+C calling");
    std::cout << "Shuting down ROS" << std::endl;

    ros::shutdown();
}

#endif
