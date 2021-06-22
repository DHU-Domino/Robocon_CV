#pragma once
#include "video.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

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

