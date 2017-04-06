//
// Created by nearlab on 06/04/17.
//

#include <ros/ros.h>
#include "ArucoBoardDetector.hpp"

int main(int argc, char** argv) {

    ros::init(argc, argv, "Aruco Extrinsic", ros::init_options::AnonymousName);

    ArucoBoardDetector abd;

    ros::spin();
}
