#include "mainPC.h"

#include <ctime>
#include <mutex>
#include <cstdio>
#include <chrono>
#include <string>
#include <vector>
#include <cstring>
#include <fstream>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <pthread.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <opencv2/opencv.hpp>

#include "TCPServer.h"
#include "WzSerialportPlus.h"
#include "CRC_Check.h"

using namespace std;
using namespace cv;

std::mutex data_mt;
Mat src, fix_img;
Mat imgHSV1, imgHSV2, res1, res2, res3, res4, mask0, mask1, mask2, mask3;
Json r;
int autoAim = 0;

extern TCPServer tcpp;
extern ProdCons_data pc_data[1];
extern WzSerialportPlus wzSerialportPlus;

volatile unsigned int prdIndex = 0;
volatile unsigned int csmIndex = 0;

Point getCenterPoint(Rect rect)
{
    Point cpt;
    cpt.x = rect.x + cvRound(rect.width / 2.0);
    cpt.y = rect.y + cvRound(rect.height / 2.0);
    return cpt;
}

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
    long long oldtime = chrono::duration_cast<chrono::seconds>(chrono::system_clock::now().time_since_epoch()).count();

    while (1)
    {
        while (prdIndex - csmIndex >= 1)
            ;
        if (a.getFrame(src))
        {
        }
        else
        {
            printf("getFrame failed!\n");
            continue;
        }
        ++prdIndex;
    }
}

void mainPC::ImageConsumer()
{
    auto a = r.add_array("categories", 20);
    auto b = r.add_array("data", 20);

    a.push_back(0);
    b.push_back(0);

    cv::dnn::Net net = cv::dnn::readNetFromDarknet("/home/domino/OpenCV_web_test/asset/yolov4-tiny-obj-rc.cfg",
                                                   "/home/domino/OpenCV_web_test/asset/yolov4-tiny-obj-rc_1000.weights");
    net.setPreferableBackend(cv::dnn::Backend::DNN_BACKEND_DEFAULT);
    net.setPreferableTarget(cv::dnn::Target::DNN_TARGET_CPU);
    std::vector<String> outNames = net.getUnconnectedOutLayersNames();
    for (int i = 0; i < outNames.size(); i++)
    {
        printf("output layer name : %s\n", outNames[i].c_str());
    }
    vector<string> classNamesVec;
    ifstream classNamesFile("/home/domino/OpenCV_web_test/asset/obj-rc.names");
    if (classNamesFile.is_open())
    {
        string className = "";
        while (std::getline(classNamesFile, className))
        {
            cout << className << endl;
            classNamesVec.push_back(className);
        }
    }

    int last_delta_x = 0;
    int64_t ms_start = now::ms();
    while (1)
    {
        while (prdIndex - csmIndex == 0)
            ;
        data_mt.lock();

        src.copyTo(fix_img);
        
        try
        {
            int64 start = getTickCount();
            Mat inputBlob = cv::dnn::blobFromImage(fix_img, 1 / 255.F, Size(416, 416), Scalar(), true, false);
            net.setInput(inputBlob);
            // 检测
            std::vector<Mat> outs;
            net.forward(outs, outNames);
            //cerr << "outs.size(): " << outs.size() << endl;
            vector<Rect> boxes;
            vector<int> classIds;
            vector<float> confidences;
            for (size_t i = 0; i < outs.size(); ++i)
            {
                // detected objects and C is a number of classes + 4 where the first 4
                float *data = (float *)outs[i].data;
                for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
                {
                    Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
                    Point classIdPoint;
                    double confidence;
                    minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
                    //cout << "confidence: " << confidence << endl;
                    if (confidence > 0.5)
                    {
                        int centerX = (int)(data[0] * fix_img.cols);
                        int centerY = (int)(data[1] * fix_img.rows);
                        int width = (int)(data[2] * fix_img.cols);
                        int height = (int)(data[3] * fix_img.rows);
                        int left = centerX - width / 2;
                        int top = centerY - height / 2;

                        classIds.push_back(classIdPoint.x);
                        confidences.push_back((float)confidence);
                        boxes.push_back(Rect(left, top, width, height));
                    }
                }
            }

            vector<int> indices;
            cv::dnn::NMSBoxes(boxes, confidences, 0.5, 0.2, indices);
            Rect midBox(1280 / 2 - 1, 1080 / 2 - 1, 2, 2);
            int distance = 1280;
            int midIndex = 0;

            tcpp.mtSerialData.lock();
            autoAim = pc_data[0].data.isAutoAim.d;
            if (autoAim != 0)
                cout << "autoAim: " << autoAim << endl;
            else
                cout << "autoAim: close\n";
            //autoAim = 1;
            tcpp.mtSerialData.unlock();

            for (size_t i = 0; i < indices.size(); ++i)
            {
                int idx = indices[i];
                Rect box = boxes[idx];
                int delta_x = abs(getCenterPoint(box).x - 1280 / 2.0);
                char rb;
                if (autoAim == 1)
                    rb = 'r';
                else if (autoAim == 2)
                    rb = 'b';
                else
                    rb = ' ';
                if (delta_x < distance && classNamesVec[classIds[idx]][0] == rb)
                {
                    distance = delta_x;
                    midBox = box;
                    midIndex = idx;
                }
            }
            if (indices.size() != 0 && autoAim != 0)
            {
                //rectangle(fix_img, midBox, Scalar(0, 0, 255), 2, 8, 0);
                
                Rect big_box(midBox.tl()-Point(15,10), midBox.br() + Point(15,10));
                rectangle(fix_img, big_box, Scalar(255, 255, 255));
                
                /*
                Mat imgHSV;
                cvtColor(src, imgHSV, COLOR_BGR2HSV);
                cvtColor(src, imgHSV1, COLOR_BGR2HSV);
                cvtColor(src, imgHSV2, COLOR_BGR2HSV);

                imgHSV1 = imgHSV1(big_box);
                imgHSV2 = imgHSV2(big_box);

                int iLowH1, iHighH1, iLowH2, iHighH2;
                int iH = imgHSV.at<Vec3b>(big_box.x, big_box.y)[0];
                int iS = imgHSV.at<Vec3b>(big_box.x, big_box.y)[1];
                int iV = imgHSV.at<Vec3b>(big_box.x, big_box.y)[2];
                int rangeH = 12, rangeS = 20, rangeV = 8;
                if(iH > 180-rangeH && iH <= rangeH)
                {
                    iLowH1 = 180 + iH - rangeH;
                    if(iLowH1 > 180){
                        iLowH1 -= 180;
                    }
                    iHighH1 = 180;
                    iLowH2 = 0;
                    iHighH2 = iH + rangeH;
                    if(iHighH2 > 180){
                        iHighH2 -= 180;
                    }
                }
                else
                {
                    iLowH1 = iH - rangeH;
                    iHighH1 = iH + rangeH;
                    iLowH2 = iH;
                    iHighH2 =iH;
                }
                cout << iH << " " << iLowH1 << " " << iHighH1 << " " << iLowH2 << " " << iHighH2 << endl;
                
                inRange(imgHSV1, Scalar(iLowH1, iS-rangeS, iV-rangeV), Scalar(iHighH1, iS+rangeS, iV+rangeV), mask0);
                inRange(imgHSV2, Scalar(iLowH2, iS-rangeS, iV-rangeV), Scalar(iHighH2, iS+rangeS, iV+rangeV), mask1);
                add(mask0, mask1, mask1);

                Mat element = getStructuringElement(MORPH_RECT, Size(9, 9));
                morphologyEx(mask1, mask2, MORPH_CLOSE, element); //闭操作

                //-----------------------------------------------------------

                vector<vector<Point>> contours;
                vector<Vec4i> hierarchy;
                findContours(mask2, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE); //查找轮廓

                Point pt[512];  //存储连通区域个数
                Moments moment; //矩
                cout << "contours.size(): " << contours.size() << endl;
                for (int i = 0; i < contours.size(); i++)
                {
                    double t = contourArea(contours.at(i));
                    cout << "areaT: " << t << endl;
                    if (10 > t)
                        continue;
                    Rect good_rect = boundingRect(contours.at(i));
                    cout << "good_rect: " << good_rect.tl()+big_box.tl() << endl;
                    rectangle(fix_img, Rect(good_rect.tl()+big_box.tl(),good_rect.br()+big_box.br()), Scalar(0, 0, 255), 2, 8, 0);
                }
                contours.clear();
                hierarchy.clear();
*/
                String className = "total:" + to_string(indices.size()) + "  " + classNamesVec[classIds[midIndex]];
                putText(fix_img, className.c_str(), midBox.tl(), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 0, 0), 2, 8);

                Send2Stm32 send_data{
                    int16_t(getCenterPoint(midBox).x - 1280 / 2.0), float(0)};
                unsigned char Tdata[9];
                Tdata[0] = 0xA5;
                Tdata[1] = 0x5A;
                Tdata[2] = send_data.delta_x_pixel.c[0];
                Tdata[3] = send_data.delta_x_pixel.c[1];
                Tdata[4] = send_data.delta_angle_yaw.c[0];
                Tdata[5] = send_data.delta_angle_yaw.c[1];
                Tdata[6] = send_data.delta_angle_yaw.c[2];
                Tdata[7] = send_data.delta_angle_yaw.c[3];
                cout << send_data.delta_x_pixel.d << endl;
                last_delta_x = send_data.delta_x_pixel.d;

                Append_CRC8_Check_Sum(Tdata, 9);
                wzSerialportPlus.send(Tdata, 9);

                a.push_back((now::ms() - ms_start));
                if (abs(send_data.delta_x_pixel.d) > 50)
                {
                    if (send_data.delta_x_pixel.d > 0)
                        b.push_back(30);
                    else
                        b.push_back(-30);
                }
                else
                {
                    b.push_back(int(send_data.delta_x_pixel.d));
                }
            }

            float fps = getTickFrequency() / (getTickCount() - start);
            float time = (getTickCount() - start) / getTickFrequency();
            ostringstream ss;
            ss << "FPS : " << fps << " detection time: " << time * 1000 << " ms";
            line(fix_img, Point2i(1280 / 2, 0), Point2i(1280 / 2, 1080), cv::Scalar(255, 0, 0), 2);
            putText(fix_img, ss.str(), Point(20, 40), 0, 1, Scalar(0, 0, 255), 2);
            putText(fix_img, "last_delta_x: " + to_string(last_delta_x), Point(20, 100), 0, 1, Scalar(0, 0, 255), 2);
        }
        catch (cv::Exception e)
        {
            cerr << e.what();
        }
        data_mt.unlock();
        ++csmIndex;
    }
}
