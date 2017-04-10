//
// Created by nearlab on 06/04/17.
//


#include <boost/range/combine.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>

#include "ArucoBoardDetector.hpp"
#include "to_string.hpp"

#include <tf_conversions/tf_kdl.h>
#include <opencv2/calib3d.hpp>

/// Constructor
ArucoBoardDetector::ArucoBoardDetector()
    : _node()
    , _private_node("~")
    , _it(_node)
{
    auto image_topic = getPrivateParam<std::string>("camera_topic");
    _camera = _it.subscribeCamera(image_topic, 1, &ArucoBoardDetector::onImageReceived, this);
    ROS_INFO_STREAM("Listening for camera images on " << image_topic.c_str());

    auto aruco_dict_id = getPrivateParam<int>("aruco_dictionary_id");
    _aruco_dict = cv::aruco::getPredefinedDictionary(
        cv::aruco::PREDEFINED_DICTIONARY_NAME(aruco_dict_id));

    std::string detections_topic;
    if (_private_node.getParam("detections_topic", detections_topic)) {
        ROS_INFO_STREAM("publishing detection images on " << detections_topic.c_str());
        _detections = _it.advertise(detections_topic, 1);
    }

    std::string camera_pose_topic = getPrivateParam<std::string>("camera_pose_topic");
    _camera_pose = _node.advertise<geometry_msgs::PoseStamped>(camera_pose_topic, 1);
    ROS_INFO_STREAM("Publishing camera pose to " << camera_pose_topic.c_str());

    _aruco_gridboard = cv::aruco::GridBoard::create(
          getPrivateParam<int>("aruco_markers_x")
        , getPrivateParam<int>("aruco_markers_y")
        , getPrivateParam<float>("aruco_markers_length")
        , getPrivateParam<float>("aruco_markers_separation")
        , _aruco_dict
    );
    _aruco_board = _aruco_gridboard.staticCast<cv::aruco::Board>();

    ROS_INFO("Aruco board detector ready.");
}

ArucoBoardDetector::~ArucoBoardDetector() {
    // Do nothing
}

/**
 * Callback called when new images are received.
 *
 * This callback performs the pose estimation using the aruco markers and publishes the pose to the
 * specified topic. Optionally it can also make an image highlighting the detected markers and their
 * transforms.
 *
 * @param img A pointer to the \c sensor_msgs::Image containing the image data
 * @param camera_info
 *      A pointer to the sensor_msgs::CameraInfo containing the camera calibrationd data.
 */
void ArucoBoardDetector::onImageReceived(
    const sensor_msgs::ImageConstPtr &img,
    const sensor_msgs::CameraInfoConstPtr &camera_info
) {
    ROS_DEBUG_STREAM("Processing image #" << img->header.seq);

    // Convert received image and camera parameters to OpenCV friendly formats

    // Can't use toCvShare since we need to convert the image
    auto cv_image = cv_bridge::toCvCopy(img, "bgr8");

    // Convert the relevant camera info parameters to cv:: equivalents.
    cv::Mat_<double> camera_matrix(3, 3, const_cast<double*>(&camera_info->K[0]));
    cv::Mat distortion_coefficients(camera_info->D);

    // Now detect the markers
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    cv::aruco::detectMarkers(cv_image->image, _aruco_dict, corners, ids, _aruco_params, rejected);

    // If no ids were detected we don't have anything else to do.
    if (ids.empty())
        return;

    // Use the markers to compute the pose.
    cv::Vec3d orientation, position;
    auto markers_used = cv::aruco::estimatePoseBoard(
        corners, ids, _aruco_board, camera_matrix, distortion_coefficients, orientation, position
    );

    // if the board was actually detected we can build the pose message
    if (markers_used > 0) {
        ROS_DEBUG_STREAM(
            "" << "Image #" << img->header.seq << ", #Markers: " << markers_used << ", O: ["
               << orientation[0] << ", " << orientation[1] << ", " << orientation[2] << "]"
               << ", P: [" << position[0] << ", " << position[1] << ", " << position[2] << "]"
        );

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = position[0];
        pose.pose.position.y = position[1];
        pose.pose.position.z = position[2];

        auto quat = rvecToQuaternion(orientation);
        pose.pose.orientation.w = quat.w;
        pose.pose.orientation.x = quat.x;
        pose.pose.orientation.y = quat.y;
        pose.pose.orientation.z = quat.z;

        _camera_pose.publish(pose);
    } else {
        ROS_WARN_STREAM("Board detection failed");
    }

    // Only generate the image showing the detections if explicitly required
    if (_detections) {
        cv::Mat new_image;
        cv_image->image.copyTo(new_image);

        cv::aruco::drawDetectedMarkers(new_image, corners, ids);

        // Estimate the pose of the single markers
        std::vector<cv::Vec3d> orientations, positions;
        cv::aruco::estimatePoseSingleMarkers(
            corners, 1.0f, camera_matrix, distortion_coefficients, orientations, positions);

                // Draw the pose of the markers
        for (const auto & zipped : boost::combine(orientations, positions)) {
            cv::aruco::drawAxis(
                new_image, camera_matrix, distortion_coefficients,
                zipped.get<0>(), zipped.get<1>(), 0.5f
            );
        }

        // Convert the cv::Mat to a sensor_msgs::Image
        sensor_msgs::Image detections_img;
        cv_bridge::CvImage(
            img->header, sensor_msgs::image_encodings::BGR8, new_image
        ).toImageMsg(detections_img);
        _detections.publish(detections_img);
    }

    // Update the count only when we're sure that we've successfully processed the image.
    _image_count += 1;
}

/**
 * Get the number of images received so far.
 * @return The number of images received so far.
 */
u_int ArucoBoardDetector::getImageCount() const {
    return _image_count;
}

/**
 * Get the latest image received.
 * @return A const reference to the latest image (read-only access).
 */
const sensor_msgs::Image &ArucoBoardDetector::getLatestImage() const {
    return _latest_image;
}


/**
 * Convert an opencv rotation vector to a quaternion.
 *
 * @note
 *  The conversion from a rotation matrix to a quaternion was lifted from glm.
 *  see: https://github.com/g-truc/glm/blob/c8ddeea744d6ea7fc3deda06bba0d1f0d2a31f6a/glm/gtc/quaternion.inl#L626
 *
 * @param rvec The rotation vector to convert
 * @return A quaternion representing the rotation.
 */
quaternion ArucoBoardDetector::rvecToQuaternion(cv::Vec3d rvec) {
    cv::Mat m(3, 3, CV_64F);
    cv::Rodrigues(rvec, m);

    quaternion quat;

    double fourXSquaredMinus1 = m.at<double>(0, 0) - m.at<double>(1, 1) - m.at<double>(2, 2);
    double fourYSquaredMinus1 = m.at<double>(1, 1) - m.at<double>(0, 0) - m.at<double>(2, 2);
    double fourZSquaredMinus1 = m.at<double>(2, 2) - m.at<double>(0, 0) - m.at<double>(1, 1);
    double fourWSquaredMinus1 = m.at<double>(0, 0) - m.at<double>(1, 1) - m.at<double>(2, 2);

    // To reduce numerical errors we need to find the largest component
    int biggestIndex = 0;
    double fourBiggestSquaredMinus1 = fourWSquaredMinus1;
    if(fourXSquaredMinus1 > fourBiggestSquaredMinus1)
    {
        fourBiggestSquaredMinus1 = fourXSquaredMinus1;
        biggestIndex = 1;
    }

    if(fourYSquaredMinus1 > fourBiggestSquaredMinus1)
    {
        fourBiggestSquaredMinus1 = fourYSquaredMinus1;
        biggestIndex = 2;
    }

    if(fourZSquaredMinus1 > fourBiggestSquaredMinus1)
    {
        fourBiggestSquaredMinus1 = fourZSquaredMinus1;
        biggestIndex = 3;
    }

    double biggestVal = sqrt(fourBiggestSquaredMinus1 + 1.0) * 0.5;
    double mult = 0.25 / biggestVal;

    switch(biggestIndex)
    {
        case 0:
        quat.w = biggestVal;
        quat.x = (m.at<double>(1, 2) - m.at<double>(2, 1)) * mult;
        quat.y = (m.at<double>(2, 0) - m.at<double>(0, 2)) * mult;
        quat.z = (m.at<double>(0, 1) - m.at<double>(1, 0)) * mult;
        break;

    case 1:
        quat.w = (m.at<double>(1, 2) - m.at<double>(2, 1)) * mult;
        quat.x = biggestVal;
        quat.y = (m.at<double>(0, 1) + m.at<double>(1, 0)) * mult;
        quat.z = (m.at<double>(2, 0) + m.at<double>(0, 2)) * mult;
        break;

    case 2:
        quat.w = (m.at<double>(2, 0) - m.at<double>(0, 2)) * mult;
        quat.x = (m.at<double>(0, 1) + m.at<double>(1, 0)) * mult;
        quat.y = biggestVal;
        quat.z = (m.at<double>(1, 2) + m.at<double>(2, 1)) * mult;
        break;

    case 3:
        quat.w = (m.at<double>(0, 1) - m.at<double>(1, 0)) * mult;
        quat.x = (m.at<double>(2, 0) + m.at<double>(0, 2)) * mult;
        quat.y = (m.at<double>(1, 2) + m.at<double>(2, 1)) * mult;
        quat.z = biggestVal;
        break;

    default:
        assert(false);
        break;
    }
    return quat;
}
