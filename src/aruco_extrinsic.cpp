//
// Created by nearlab on 06/04/17.
//

#include <iostream>

#include <ros/ros.h>
#include "ArucoBoardDetector.hpp"

int main(int argc, char** argv) {

    ros::init(argc, argv, "aruco_extrinsic", ros::init_options::AnonymousName);

    ArucoBoardDetector abd;

    ros::spin();

}
