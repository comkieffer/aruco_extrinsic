//
// Created by nearlab on 06/04/17.
//

#include <boost/range/combine.hpp>

#include <cv_bridge/cv_bridge.h>

#include "ArucoBoardDetector.hpp"

ArucoBoardDetector::ArucoBoardDetector()
    : _node()
    , _private_node("~")
    , _it(_node)
{
    auto image_topic = getPrivateParam<std::string>("camera_topic");
    _camera = _it.subscribeCamera(image_topic, 1, &ArucoBoardDetector::onImageReceived, this);

    auto aruco_dict_id = getPrivateParam<int>("aruco_dictionary_id");
    _aruco_dict = cv::aruco::getPredefinedDictionary(
        cv::aruco::PREDEFINED_DICTIONARY_NAME(aruco_dict_id));

}

ArucoBoardDetector::~ArucoBoardDetector() {
    // Do nothing
}

void ArucoBoardDetector::onImageReceived(
    const sensor_msgs::ImageConstPtr &img,
    const sensor_msgs::CameraInfoConstPtr &camera_info
) {
    // Convert things to opencv
    auto cv_image = cv_bridge::toCvShare(img);

    cv::Mat camera_matrix(3, 3, CV_64F);
    std::copy(camera_info->K.begin(), camera_info->K.end(), camera_matrix.data);

    cv::Mat distortion_coefficients(camera_info->D);

    // Now detect the markers
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;

    cv::aruco::detectMarkers(cv_image->image, _aruco_dict, corners, ids, _aruco_params, rejected);

    // If no ids were detected we don't have anything else to do.
    if (ids.empty())
        return;

    std::vector<cv::Vec3d> orientations, translations;
    cv::aruco::estimatePoseSingleMarkers(
        corners, 1.0f, camera_matrix, distortion_coefficients, orientations, translations);

    if (!ids.empty()) {
        cv::Mat new_image;
        cv_image->image.copyTo(new_image);

        cv::aruco::drawDetectedMarkers(new_image, corners, ids);

        for (const auto & zipped : boost::combine(orientations, translations)) {
            cv::aruco::drawAxis(
                new_image, camera_matrix, distortion_coefficients,
                zipped.get<0>(), zipped.get<1>(), 1
            );
        }

        cv::imshow("detections", new_image);
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
