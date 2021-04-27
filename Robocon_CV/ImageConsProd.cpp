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
    Mat copyROI;

    while(1){
        while(prdIdx - csmIdx == 0);
        datadata[csmIdx % BUFFER_SIZE].img.copyTo(src);
        datadata[csmIdx % BUFFER_SIZE].img.copyTo(src1);
        if(src.empty()){
            continue;
        }
        if (src.channels() != 3)
            continue;

        //红色
        //int iLowH1 = int((356 / 360.0) * 180), iHighH1 = int((360 / 360.0) * 180);
        int iLowH2 = int((1 / 360.0) * 180), iHighH2 = int((10 / 360.0) * 180);
        int iLowS = int(255 * 0.55), iHighS = int(255 * 0.90);
        int iLowV = int(255 * 0.20), iHighV = int(255 * 1.00);

        Mat imgHSV1, imgHSV2, res1, res2, res3, res4, mask0, mask1, mask2, mask3;
        cvtColor(src, imgHSV1, COLOR_BGR2HSV);
        cvtColor(src, imgHSV2, COLOR_BGR2HSV);
        //inRange(imgHSV1, Scalar(iLowH1, iLowS, iLowV), Scalar(iHighH1, iHighS, iHighV), mask0);
        inRange(imgHSV2, Scalar(iLowH2, iLowS, iLowV), Scalar(iHighH2, iHighS, iHighV), mask1);

        //add(mask0, mask1, mask1);

        bitwise_and(src, src, res1, mask1);
        imshow_("mask1", res1, isShow);

        morphologyEx(mask1, mask2, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(15, 15)));//开操作
        bitwise_and(src, src, res2, mask2);
        imshow_("mask2", res2, isShow);

        morphologyEx(mask2, mask3, MORPH_CLOSE, getStructuringElement(MORPH_RECT, Size(200, 200)));
        bitwise_and(src, src, res3, mask2);
        imshow_("mask3", res3, isShow);

        vector<vector <Point> >contours;
        vector<Vec4i>hierarchy;
        findContours(mask3, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);//查找轮廓

        Point pt[512];//存储连通区域个数
        Moments moment;//矩

        //读取每一个轮廓求面积，最大的部分截掉
        if (mInit != 2) {
            for (int i = 0; i < contours.size(); i++) {
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
                    cv::Rect rect;
                    rect.width = VIDEO_WIDTH;
                    rect.height = maxCoor.y - 200;
                    rect.x = 0;
                    rect.y = 0;
                    roiMask = mask2(rect);
                    //copyROI = mask3(rect);
                    imshow_("haha", roiMask, isShow);
                    mInit = 2;
                }
            }
            mInit = 1;
        }
        contours.clear(); hierarchy.clear();
        //imshow_("roiMask", roiMask, isShow);
        /*
        float line[3];

        if (RansacLine(roiMask, line))
        {
            cv::Point point1, point2;
            if (line[0] == 0 && line[1] != 0)
            {
                point1 = cv::Point(0, -1 * line[2] / line[1]);
                point2 = cv::Point(copyROI.cols - 1, -1 * line[2] / line[1]);
            }
            else if (line[0] != 0 && line[1] == 0)
            {
                point1 = cv::Point(-1 * line[2] / line[0], 0);
                point2 = cv::Point(-1 * line[2] / line[0], copyROI.rows - 1);
            }
            else
            {
                point1 = cv::Point(0, -1 * line[2] / line[1]);
                point2 = cv::Point(copyROI.cols, -1 * (copyROI.cols * line[0] + line[2]) / line[1]);

            }
            std::cout << "line: " << line << std::endl;
            cv::line(copyROI, point1, point2, cv::Scalar(0, 255, 0));
            cv::namedWindow("Ransac line", 0);
            imshow("Ransac line", copyROI);
            cv::waitKey(0);
        }
        else
        {
            std::cout << "寻找直线失败，请确认图像，并调整提取前景的阈值" << std::endl;
        }*/


        vector<RotatedRect> vrrect;
        findContours(roiMask, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);//查找轮廓
        for (int i = 0; i < contours.size(); i++) { //读取每一个轮廓求取重心
            Mat temp(contours.at(i));
            
            double t = contourArea(contours.at(i));
            if (t < 1000 || t >15000) {
                continue;
            }
            cout << t << " ";


            RotatedRect _rect = minAreaRect(contours.at(i));
            RotatedRect rect(_rect.center, _rect.size + Size2f(20, 10), _rect.angle);
            
            vrrect.push_back(rect);
            //Rect perROI(rect);
            Point2f p[4];
            rect.points(p);
            line(src, p[0], p[1], Scalar(0, 255, 0), 2);
            line(src, p[1], p[2], Scalar(0, 255, 0), 2);
            line(src, p[2], p[3], Scalar(0, 255, 0), 2);
            line(src, p[3], p[0], Scalar(0, 255, 0), 2);
            
            imshow_("imgW", src, isShow);
        }
        cout << endl;
        /*
        while (vrrect.size() > 4) {//大临小，暴力匹配
            int u = 0;
            for (int i = 0; i < vrrect.size(); ++i) {
                for (int j = i; j < vrrect.size(); ++j) {
                    if (i == j) continue;
                    double dis = pow((vrrect.at(i).center.x - vrrect.at(j).center.x), 2) + pow((vrrect.at(i).center.y - vrrect.at(j).center.y), 2);
                    //cout << " i " << i << ",j" << j << ": "<< dis << " ";
                    if (dis < 5000) {
                        
                        u = 1;
                        break;
                    }
                }
                if (u == 1)
                    break;
            }
        }*/
        
        
        imshow("out", src1);
        ++csmIdx;
        waitKey(1);
        /*
            moment = moments(temp, false);
            if (moment.m00 != 0) {//除数不能为0
                pt[i].x = cvRound(moment.m10 / moment.m00);//计算重心横坐标
                pt[i].y = cvRound(moment.m01 / moment.m00);//计算重心纵坐标
                if (minDx > (pt[i].x - VIDEO_WIDTH / 2.f)) {
                    minDx = (pt[i].x - VIDEO_WIDTH / 2.f);
                    midBucket = pt[i];
                }
            }
        */
    }
}

bool ImageConsProd::RansacLine(cv::Mat& img_bin, float* line)
{
    if (img_bin.empty() || img_bin.channels() != 1)
        return false;
    std::vector<cv::Point> points;
    for (int i = 0; i < img_bin.rows; i++)
    {
        uchar* pb = img_bin.ptr<uchar>(i);
        for (int j = 0; j < img_bin.cols; j++)
        {
            if (pb[j] > 0)//前景，根据情况修改条件，收集所有目标点
                points.push_back(cv::Point(j, i));
        }
    }
    if (points.size() == 0)
        return false;
    int distance_thres = 2;//满足集内条件，即以当前点到直线距离不大于distance_thres认定为该点在直线上。
    int loop_count = 0;
    int iner_count = 0;
    int max_iner_count = 0;
    float distance = 0;
    float sqrt_ = 0;
    int rand_k1 = 0, rand_k2 = 0;
    cv::Point point1, point2;
    float A = 0, B = 0, C = 0;
    float max_dis = 0, min_dis = 100000, average_dis = 0;
    while (loop_count++ < 10000)//最大循环次数，根据情况可设定为所有点数的10倍
    {
        srand((unsigned int)time(0));
        rand_k1 = rand() % points.size();
        point1 = points[rand_k1];
        rand_k2 = rand() % points.size();
        while (rand_k1 == rand_k2)
            rand_k2 = rand() % points.size();
        point2 = points[rand_k2];
        A = point2.y - point1.y;
        B = point1.x - point2.x;
        C = point1.x * (point1.y - point2.y) + point1.y * (point2.x - point1.x);
        sqrt_ = sqrtf(A * A + B * B);
        iner_count = 0;
        float max_dis_ = 0, min_dis_ = 100000, sum_dis = 0;
        for (int i = 0; i < points.size(); i++)
        {
            distance = abs(points[i].x * A + points[i].y * B + C) * 1.0 / sqrt_;
            if (distance < distance_thres)
                iner_count++;
            if (max_dis_ < distance)
                max_dis_ = distance;
            if (min_dis_ > distance)
                min_dis_ = distance;
            sum_dis += distance;
        }
        if (iner_count >= max_iner_count)
        {
            max_iner_count = iner_count;
            line[0] = A;
            line[1] = B;
            line[2] = C;
            max_dis = max_dis_;
            min_dis = min_dis_;
            average_dis = sum_dis / points.size();
        }
    }
    std::cout << "Ransac line 最大误差：" << max_dis << " 最小误差：" << min_dis << " 平均误差：" << average_dis << std::endl;
    return true;
}

void ImageConsProd::imshow_(string winName, Mat m, bool isShow) {
    if (isShow)
        imshow(winName, m);
}