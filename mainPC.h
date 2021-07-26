#pragma once
#include "video.h"
#include "kalman.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include <list>
#include <ctime>
#include <mutex>
#include <thread>
#include <cstdio>
#include <chrono>
#include <string>
#include <vector>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <iostream>

#include "co/co.h"
#include "co/so.h"
#include "co/log.h"
#include "co/time.h"
#include "co/json.h"
#include "co/thread.h"

#include <sys/types.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>

#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <netinet/in.h>

const int RED = 0;
const int BLUE = 1;
const int ALL = 2;

typedef struct detectAns
{
    Rect midBox;
    int distance = 1280;
    int midIndex = -1;
}temp_ans;

Point getMoments(Mat, Rect);

class mainPC
{
public:
    int setting;
    
    mainPC(int * setting);
    void ImageProducer();
    void ImageConsumer();
    void switchModel(int);
    cv::VideoWriter initVideoWriter(std::string);
    
private:
    cv::dnn::Net net;
    std::ifstream classNamesFile;
    std::vector<cv::String> outNames;
    std::vector<std::string> classNamesVec;

    video a;
    cv::VideoWriter video_writer_src;
    cv::VideoWriter video_writer_fix;
    kalman kal;
    int color = 0;
    int target_control = -1;
    int position_control = -1;
    int deltaX[3][5];
    int ExposeTime;
    int saveDebug;
    int saveRecord;
};

