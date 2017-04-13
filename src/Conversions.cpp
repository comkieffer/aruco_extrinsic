//
// Created by nearlab on 13/04/17.
//

#include <opencv2/calib3d.hpp>

#include "Conversions.hpp"

/**
 * Convert an OpenCV `rvec`, `tvec` pair to a ROS `Pose`
 *
 * @param position The OpenCV 'tvec`
 * @param orientation The OpenCV `rvec`
 * @return A ROS `Pose` object
 */
geometry_msgs::Pose to_pose(cv::Vec3d position, cv::Vec3d orientation) {
    geometry_msgs::Pose pose;
    pose.position.x = position[0];
    pose.position.y = position[1];
    pose.position.z = position[2];

    auto quat = rvecToQuaternion(orientation);
    pose.orientation.w = quat.w;
    pose.orientation.x = quat.x;
    pose.orientation.y = quat.y;
    pose.orientation.z = quat.z;

    return pose;
}

/**
 * Convert an OpenCV `rvec`, `tvec` pair to a ROS `PoseStamped`
 *
 * @param position The OpenCV 'tvec`
 * @param orientation The OpenCV `rvec`
 * @return A ROS `PoseStamped` object
 */
geometry_msgs::PoseStamped to_pose_stamped(
    const std_msgs::Header &header,
    const cv::Vec3d &position,
    const cv::Vec3d &orientation
) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = header;
    pose_stamped.pose = to_pose(position, orientation);

    return pose_stamped;
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
quaternion rvecToQuaternion(cv::Vec3d rvec) {
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

