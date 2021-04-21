#include "ImageConsProd.hpp"
#include "iostream"
#include "GenICam/System.h"
#include "GenICam/Camera.h"
#include "GenICam/GigE/GigECamera.h"
#include "GenICam/GigE/GigEInterface.h"
#include "Infra/PrintLog.h"
#include "Memory/SharedPtr.h"

using namespace cv;
using namespace std;
using namespace Eigen;

using namespace Dahua::GenICam;
using namespace Dahua::Infra;

#define VIDEO_WIDTH  1280
#define VIDEO_HEIGHT 720

#define EXPOSURE 30000// 5000
#define BUFFER_SIZE 1

#define CONNECT_CLIENT

const int useCam = 0;
const bool isShow = true;

volatile unsigned int prdIdx;
volatile unsigned int csmIdx;

struct ImageData {
    Mat img;
    unsigned int frame;
    long long ts;
};
ImageData datadata[BUFFER_SIZE];

Eigen::Vector3d ImageConsProd::euler_angles, ImageConsProd::old_euler_angles;
Eigen::Vector3d ImageConsProd::T_n, ImageConsProd::old_T_n;
long long ImageConsProd::old_ts, ImageConsProd::ts;


void ImageConsProd::ImageProducer(){

    compression_params.push_back(IMWRITE_PNG_COMPRESSION);      //选择PNG格式
    compression_params.push_back(0);                            //无压缩png（从0-9.较高的值意味着更小的尺寸和更长的压缩时间而默认值是3.本人选择0表示不压缩）

    Video v;
    v.setVideoMode(Video::VideoType::DAHUA);
    if (!v.videoCheck())
    {
        printf("videoCheck failed!\n");
        return ;
    }
    if (!v.videoOpen())
    {
        printf("videoOpen failed!\n");
        return ;
    }
    int fps = 200;                                              //获取摄像机帧率
    v.SetExposeTime(EXPOSURE);                                  //设置曝光
    v.setFrameRate(fps);
    v.setBalanceRatio(1.39, 1, 1.53);
    Video::ETrigType type = Video::ETrigType::trigContinous;    //改为连续拉流
    v.CameraChangeTrig(type);                                   //默认为软触发
    if (!v.videoStart()) {
        printf("videoStart failed!\n");
        return ;
    }
    

    Mat img;
    while(1){
        while(prdIdx - csmIdx >= BUFFER_SIZE);
        ///*
        v.startGrabbing();
        if (v.getFrame(img)){
        }
        else{
            printf("getFrame failed!\n");
            continue ;
        }
        datadata[prdIdx % BUFFER_SIZE].img = img;
        datadata[prdIdx % BUFFER_SIZE].frame++;
        auto t = chrono::duration_cast<chrono::nanoseconds>(chrono::system_clock::now().time_since_epoch());
        long long t_count = t.count();
        datadata[prdIdx % BUFFER_SIZE].ts = t_count;
        ++prdIdx;
    }
}

void ImageConsProd::ImageConsumer(){
    
    FileStorage fs1("C:/Users/49673/Documents/GitHub/Graduation-project/calibration/calibration/out_camera_data.yml", FileStorage::READ);
    if (!fs1.isOpened()) {
        cout << "Could not open the configuration file" << endl;
        return;
    }

    Mat cam_matrix_720, distortion_coeff_720;
    fs1["camera_matrix"] >> cam_matrix_720;

    Mat src, src1;

    int mInit = 0;
    int maxArea = -1;
    Point maxCoor;
    Mat roiMask;

    while(1){
        while(prdIdx - csmIdx == 0);
        datadata[csmIdx % BUFFER_SIZE].img.copyTo(src);
        datadata[csmIdx % BUFFER_SIZE].img.copyTo(src1);
        if(src.empty()){
            continue;
        }
        if (src.channels() != 3)
            continue;
        //flip(src, src, 1);
        //imshow_("src", src, isShow);

        /*
        //粉色
        int iLowH1 = int((320 / 360.0) * 180), iHighH1 = int((360 / 360.0) * 180);
        int iLowH2 = int((0 / 360.0) * 180), iHighH2 = int((3 / 360.0) * 180);
        int iLowS = int(255 * 0.40), iHighS = int(255 * 0.70);
        int iLowV = int(255 * 0.70), iHighV = int(255 * 1.00);
        */
        //红色
        int iLowH1 = int((354 / 360.0) * 180), iHighH1 = int((360 / 360.0) * 180);
        int iLowH2 = int((0 / 360.0) * 180), iHighH2 = int((10 / 360.0) * 180);
        int iLowS = int(255 * 0.55), iHighS = int(255 * 0.90);
        int iLowV = int(255 * 0.20), iHighV = int(255 * 1.00);

        Mat imgHSV1, imgHSV2, res1, res2, res3, res4, mask0, mask1, mask2, mask3;
        cvtColor(src, imgHSV1, COLOR_BGR2HSV);
        cvtColor(src, imgHSV2, COLOR_BGR2HSV);
        inRange(imgHSV1, Scalar(iLowH1, iLowS, iLowV), Scalar(iHighH1, iHighS, iHighV), mask0);
        inRange(imgHSV2, Scalar(iLowH2, iLowS, iLowV), Scalar(iHighH2, iHighS, iHighV), mask1);

        add(mask0, mask1, mask1);

        bitwise_and(src, src, res1, mask1);
        imshow_("mask1", res1, isShow);

        Mat element = getStructuringElement(MORPH_RECT, Size(11, 11));
        Mat element1 = getStructuringElement(MORPH_RECT, Size(11, 11));
        morphologyEx(mask1, mask2, MORPH_OPEN, element);//开操作
        bitwise_and(src, src, res2, mask2);
        imshow_("mask2", res2, isShow);

        morphologyEx(mask2, mask3, MORPH_CLOSE, element1);//闭操作
        bitwise_and(src, src, res3, mask2);
        imshow_("mask3", res3, isShow);


        //-----------------------------------------------------------
        

        vector<vector <Point> >contours;
        vector<Vec4i>hierarchy;
        findContours(mask3, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);//查找轮廓

        Point pt[512];//存储连通区域个数
        Moments moment;//矩

        if (mInit != 2) {
            for (int i = 0; i < contours.size(); i++) { //读取每一个轮廓求取重心

                if (mInit == 0) {
                    double t = contourArea(contours.at(i));
                    if (maxArea < t) {
                        maxArea = t;
                        Mat temp(contours.at(i));
                        moment = moments(temp, false);
                        maxCoor.x = cvRound(moment.m10 / moment.m00);
                        maxCoor.y = cvRound(moment.m01 / moment.m00);
                    }
                }
                else if (mInit == 1) {
                    //Point p = Point(maxCoor.x, maxCoor.y);//重心坐标
                    //circle(src, p, 1, Scalar(255, 255, 255), 5, 8);//原图画出重心坐标
                    //imshow_("haha", src, isShow);
                    //waitKey(1);
                    //cout << maxCoor << endl;
                    cv::Rect rect;
                    rect.width = VIDEO_WIDTH;
                    rect.height = maxCoor.y - 200;
                    rect.x = 0;
                    rect.y = 0;
                    roiMask = mask3(rect);
                    imshow_("haha", roiMask, isShow);
                    mInit = 2;
                }
            }
            mInit = 1;
        }
        
        contours.clear(); hierarchy.clear();
        //imshow_("roiMask", roiMask, isShow);

        findContours(roiMask, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);//查找轮廓
        Point2d midBucket;
        double minDx = 10000;
        for (int i = 0; i < contours.size(); i++) { //读取每一个轮廓求取重心
            Mat temp(contours.at(i));
            moment = moments(temp, false);
            if (moment.m00 != 0) {//除数不能为0
                pt[i].x = cvRound(moment.m10 / moment.m00);//计算重心横坐标
                pt[i].y = cvRound(moment.m01 / moment.m00);//计算重心纵坐标
                if (minDx > (pt[i].x - VIDEO_WIDTH / 2.f)) {
                    minDx = (pt[i].x - VIDEO_WIDTH / 2.f);
                    midBucket = pt[i];
                }
            }
            Point p = Point(pt[i].x, pt[i].y);//重心坐标
            circle(src, p, 1, Scalar(255, 255, 255), 5, 8);//原图画出重心坐标
            imshow_("imgW", src, isShow);
        }
        cout << midBucket << endl;
        //cout << endl;

        //-----------------------------------------------------------

        
        imshow("out", src1);
        ++csmIdx;
        waitKey(1);

    }
}

void ImageConsProd::imshow_(string winName, Mat m, bool isShow) {
    if (isShow)
        imshow(winName, m);
}