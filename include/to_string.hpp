//
// Created by nearlab on 10/04/17.
//

#pragma once

#include <vector>
#include <string>
#include <sstream>

#include <boost/array.hpp>

template <class T>
std::string to_string(std::vector<T> in) {
    std::ostringstream oss;

    oss << "[";
    for (auto i = 0; i < in.size()-1; i++) {
        oss << in[i] << ", ";
    }
    oss << in.back() << "]";

    return oss.str();
}

template<class T, std::size_t N>
std::string to_string(boost::array<T, N> in) {
    std::ostringstream oss;

    oss << "[";
    for (auto i = 0; i < in.size()-1; i++) {
        oss << in[i] << ", ";
    }
    oss << in.back() << "]";

    return oss.str();
}

inline std::string to_string(cv::Mat in) {
    std::ostringstream oss;
    oss << in;

    return oss.str();
}


