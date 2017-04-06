//
// Created by nearlab on 06/04/17.
//

#pragma once

#include <string>
#include <boost/exception/all.hpp>

class ErrorBase : public virtual boost::exception,
    public virtual std::exception
{
};

typedef boost::error_info<struct tag_missing_ros_param, const std::string> missing_ros_param;
class RosParamError : public virtual ErrorBase {};
