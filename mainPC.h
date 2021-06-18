#pragma once
#include "video.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include "co/co.h"
#include "co/log.h"
#include "co/time.h"
#include "co/thread.h"

class mainPC
{
public:
    int setting;
    
    mainPC(int * setting);
    void ImageProducer();
    void ImageConsumer();
    
private:
    video a;
    int ROI[4];
    int ExposeTime;
    double AdjustPlus;
    double BalanceRatio;
    double FrameRate;
};

