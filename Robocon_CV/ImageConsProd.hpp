#pragma once

#include <cmath>
#include <random>
#include <string>
#include <cstdlib>
#include <fstream>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/opencv.hpp"

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "opencv2/core/eigen.hpp"

#include "video.h"
#include "TCPServer.h"

extern bool isRecord;

class ImageConsProd {
public:
    int _a;
    //CarPnP car;
    std::vector<int> compression_params;
    cv::VideoWriter vw;

    static Eigen::Vector3d euler_angles, old_euler_angles;
    static Eigen::Vector3d T_n, old_T_n;
    static long long old_ts, ts;

    ImageConsProd(int a) {
        _a = a;
    }
    void ImageProducer();
    void ImageConsumer();

    void imshow_(std::string winName, cv::Mat m, bool isShow);
};


