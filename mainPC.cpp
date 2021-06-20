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
                //cout << "distance: " << distance << " indices.size():" << indices.size() << " x:" << midBox.x << " y:" << midBox.y << endl;
                rectangle(fix_img, midBox, Scalar(0, 0, 255), 2, 8, 0);
                Point boxCenter(midBox.tl().x + midBox.width/2, midBox.tl().y + midBox.height/2);
                circle(fix_img, boxCenter, 10, Scalar(255, 255, 255));

                Mat imgHSV;
                cvtColor(fix_img, imgHSV, COLOR_BGR2HSV);

                double R = 100;
                double angle = 30;
                double h = R * cos(angle / 180 * 3.1415);
                double r = R * sin(angle / 180 * 3.1415);

                double H1 = imgHSV.at<Vec3b>(boxCenter.x, boxCenter.y)[0] * 2;
                double S1 = imgHSV.at<Vec3b>(boxCenter.x, boxCenter.y)[1] / 255.0;
                double V1 = imgHSV.at<Vec3b>(boxCenter.x, boxCenter.y)[2] / 255.0;
                double x1 = r * V1 * S1 * cos(H1);
                double y1 = r * V1 * S1 * sin(H1);
                double z1 = h * (1 - V1);
                
                Point boxDelta(midBox.tl().x - midBox.width/2, midBox.tl().y + midBox.height/2);
                circle(fix_img, boxDelta, 10, Scalar(255, 255, 255));
                double H2 = imgHSV.at<Vec3b>(boxDelta.x, boxDelta.y)[0] * 2;
                double S2 = imgHSV.at<Vec3b>(boxDelta.x, boxDelta.y)[1] / 255.0;
                double V2 = imgHSV.at<Vec3b>(boxDelta.x, boxDelta.y)[2] / 255.0;
                double x2 = r * V2 * S2 * cos(H2);
                double y2 = r * V2 * S2 * sin(H2);
                double z2 = h * (1 - V2);

                cout << sqrt(pow(x1-x2,2)+pow(y1-y2,2)+pow(z1-z2,2)) << endl;

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
                    if(send_data.delta_x_pixel.d > 0)
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
