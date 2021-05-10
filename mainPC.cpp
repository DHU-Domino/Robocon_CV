#include "mainPC.h"

#include <ctime>
#include <mutex>
#include <cstdio>
#include <chrono>
#include <string>
#include <vector>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <pthread.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

std::mutex data_mt;
Mat img;

volatile unsigned int prdIndex = 0;
volatile unsigned int csmIndex = 0;

mainPC::mainPC(int *setting)
{
    FileStorage storage("setting.xml", FileStorage::READ);
    ROI[0] = storage["OffSetX"].real();
    ROI[1] = storage["OffSetY"].real();
    ROI[2] = storage["Width"].real();
    ROI[3] = storage["Height"].real();
    ExposeTime = storage["ExposeTime"].real();
    AdjustPlus = storage["AdjustPlus"].real();
    BalanceRatio = storage["BalanceRatio"].real();
    FrameRate = storage["FrameRate"].real();
}

void mainPC::ImageProducer()
{
    if (!a.videoCheck())
    {
        cerr << "没相机\n";
    }
    if (a.videoOpen())
    {
        a.streamControl(1);
        cerr << "ok\n";
    }
    
    while (1)
    {
        while (prdIndex - csmIndex >= 1);
        data_mt.lock();
        if (a.getFrame(img))
        {

        }
        else
        {
            printf("getFrame failed!\n");
            continue;
        }
        data_mt.unlock();
        ++prdIndex;
    }
}

void mainPC::ImageConsumer()
{
    Mat src, src1;
    while (1)
    {
        while (prdIndex - csmIndex == 0);
        
        Mat src;
        img.copyTo(src);
        
        //imshow("out", src);
        //waitKey(1);

        ++csmIndex;
    }
}
