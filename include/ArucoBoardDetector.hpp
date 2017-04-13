//
// Created by nearlab on 06/04/17.
//


/**
  * @file
  *
  */

#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <opencv2/aruco.hpp>

#include "Exceptions.hpp"



class ArucoBoardDetector
{
public:
    ArucoBoardDetector();
    virtual ~ArucoBoardDetector();

    void onImageReceived(
        const sensor_msgs::ImageConstPtr &img,
        const sensor_msgs::CameraInfoConstPtr &camera_info
    );

    u_int getImageCount() const;

    const sensor_msgs::Image & getLatestImage() const;

protected:
    template <class T>
    T getPrivateParam(std::string name);

private:

    ros::NodeHandle _node;
    ros::NodeHandle _private_node;

    /// Publisher used to publish the camera pose information as a \sensor_msgs::PoseStamped message
    ros::Publisher _camera_pose;

    image_transport::ImageTransport _it;

    /// Subscribes to the camera images
    image_transport::CameraSubscriber _camera;

    /// Publishes an image with all the detected aruco markers highlighted
    image_transport::Publisher _detections;

    ros::Publisher _markers;

    /// Latest image received. May be empty if no images have been received.
    sensor_msgs::Image _latest_image;

    /// Parameters required for the aruco
    cv::Ptr<cv::aruco::Dictionary> _aruco_dict;
    cv::Ptr<cv::aruco::DetectorParameters> _aruco_params = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::GridBoard> _aruco_gridboard;
    cv::Ptr<cv::aruco::Board> _aruco_board;

    /// Number of images received so far
    u_int _image_count = 0;
};

/**
 * Get a parameter from the private namespace of a node.
 *
 * If the parameter is not set a message is printed with \c ROS_ERROR and an exception of type
 * #RosParamError is thrown.
 *
 * @tparam T The type of the value to query. This is also the type of the returned value.
 * @param name The name of the parameter to query.
 * @return The value of the parameter.
 */
template <class T>
T ArucoBoardDetector::getPrivateParam(std::string name) {
    T value;
    if (!_private_node.getParam(name, value)) {
        ROS_ERROR("Unable to retrieve parameter <%s>.", _private_node.resolveName(name).c_str());
        throw RosParamError() << missing_ros_param(name);
    }

    return value;
}




