//
// Created by nearlab on 13/04/17.
//

#pragma once

#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <opencv2/core.hpp>


geometry_msgs::Pose to_pose(cv::Vec3d position, cv::Vec3d orientation);

geometry_msgs::PoseStamped to_pose_stamped(
    const std_msgs::Header &header,
    const cv::Vec3d &position,
    const cv::Vec3d &orientation
);

/// Simple struct to represent a quaternion.
struct quaternion { double x, y, z, w; };

quaternion rvecToQuaternion(cv::Vec3d rvec);
