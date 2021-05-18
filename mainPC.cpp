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

using namespace std;
using namespace cv;

std::mutex data_mt;
Mat src, fix_img;

volatile unsigned int prdIndex = 0;
volatile unsigned int csmIndex = 0;

Point getCenterPoint(Rect rect)
{
    Point cpt;
    cpt.x = rect.x + cvRound(rect.width/2.0);
    cpt.y = rect.y + cvRound(rect.height/2.0);
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
            long long newtime = chrono::duration_cast<chrono::seconds>(chrono::system_clock::now().time_since_epoch()).count();
            if(newtime - oldtime > 5){
                string t = "/home/domino/domino_dataset/"+to_string(newtime)+".bmp";
                imwrite(t.c_str(), src);
                oldtime = newtime;
                cout << "photo\n";
            }
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
    cv::dnn::Net net = cv::dnn::readNetFromDarknet("/home/domino/OpenCV_web_test/asset/yolov4-tiny-obj-rc.cfg",
                                                   "/home/domino/OpenCV_web_test/asset/yolov4-tiny-obj-rc_best.weights");
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
            Rect midBox(1280/2-1,1080/2-1,2,2);
            int distance = 1280;
            int midIndex = 0;
            for (size_t i = 0; i < indices.size(); ++i)
            {
                int idx = indices[i];
                Rect box = boxes[idx];
                int delta_x = abs(getCenterPoint(box).x - 1280/2.0);
                if(delta_x < distance){
                    distance = delta_x;
                    midBox = box;
                    midIndex = idx;
                }
            }
            if(indices.size() != 0){
                //cout << "distance: " << distance << " indices.size():" << indices.size() << " x:" << midBox.x << " y:" << midBox.y << endl;
                rectangle(fix_img, midBox, Scalar(0, 0, 255), 2, 8, 0);
                String className = classNamesVec[classIds[midIndex]];
                putText(fix_img, className.c_str(), midBox.tl(), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 0, 0), 2, 8);
            }
            

            float fps = getTickFrequency() / (getTickCount() - start);
            float time = (getTickCount() - start) / getTickFrequency();
            ostringstream ss;
            ss << "FPS : " << fps << " detection time: " << time * 1000 << " ms";
            putText(fix_img, ss.str(), Point(20, 40), 0, 1, Scalar(0, 0, 255));
            
            
        }
        catch (cv::Exception e)
        {
            cerr << e.what();
        }
        data_mt.unlock();
        ++csmIndex;
    }
}
