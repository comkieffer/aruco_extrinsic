//
// Created by nearlab on 06/04/17.
//


#include <boost/range/combine.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <aruco_extrinsic/ArucoMarkers.h>

#include "ArucoBoardDetector.hpp"
#include "to_string.hpp"
#include "Conversions.hpp"


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

    std::string markers_topic;
    if (_private_node.getParam("markers_topic", markers_topic)) {
        ROS_INFO_STREAM("Publishing detected marker poses and ids on " << markers_topic.c_str());
        _markers = _node.advertise<aruco_extrinsic::ArucoMarkers>(markers_topic, 1);
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
    cv::Vec3d orientation{0, 0, 0}, position{0, 0, 0};
    auto markers_used = cv::aruco::estimatePoseBoard(
        corners, ids, _aruco_board, camera_matrix, distortion_coefficients, orientation, position
    );

    // if the board was actually detected we can build the pose message
    if (markers_used > 0) {
        ROS_DEBUG_STREAM(
            "" << "Image #" << img->header.seq << ", #Markers: " << markers_used << ", "
                << "P: [" << position[0] << ", " << position[1] << ", " << position[2] << "]"
                << "O: [" << orientation[0] << ", " << orientation[1] << ", " << orientation[2] << "], "
        );

        auto pose = to_pose_stamped(img->header, position, orientation);
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

    // If the markers topic name has been set we publish the pose of all the markers
    if (_markers) {
        aruco_extrinsic::ArucoMarkers markers;
        markers.header = img->header;

        // Estimate the pose of the single markers
        std::vector<cv::Vec3d> orientations, positions;
        cv::aruco::estimatePoseSingleMarkers(
            corners, 1.0f, camera_matrix, distortion_coefficients, orientations, positions);

        for (auto marker_info : boost::combine(ids, positions, orientations)) {
            aruco_extrinsic::ArucoMarker marker;
            marker.id = marker_info.get<0>();
            marker.pose = to_pose(marker_info.get<1>(), marker_info.get<2>());

            markers.markers.push_back(marker);
        }

        _markers.publish(markers);
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

