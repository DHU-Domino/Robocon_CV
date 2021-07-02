#include "mainPC.h"

#include "WzSerialportPlus.h"
#include "CRC_Check.h"

using namespace std;
using namespace cv;

std::mutex data_mt, scrLock;
Mat src, fix_img;
Json aim, chassis;
int autoAim = 0;

extern WzSerialportPlus wzSerialportPlus;
extern SendData newSerialData;
extern mutex newSerialDataLock;

volatile unsigned int prdIndex = 0;
volatile unsigned int csmIndex = 0;

Point getCenterPoint(Rect rect)
{
    Point cpt;
    cpt.x = rect.x + cvRound(rect.width / 2.0);
    cpt.y = rect.y + cvRound(rect.height / 2.0);
    return cpt;
}
Point rangeLimitPoint(Point p)
{
    if (p.x < 0)
        p.x = 0;
    if (p.y < 0)
        p.y = 0;
    if (p.x > 1280)
        p.x = 1280;
    if (p.y > 1024)
        p.y = 1024;
    return p;
}
Rect rangeLimitRect(Rect r)
{
    if (r.x < 0)
        r.x = 0;
    if (r.y < 0)
        r.y = 0;
    if (r.x > 1279)
        r.x = 1279;
    if (r.y > 1023)
        r.y = 1023;
    if (r.x + r.width > 1280)
        r.width = 1280 - r.x;
    if (r.y + r.height > 1024)
        r.height = 1024 - r.y;
    return r;
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
    while (1)
    {
        while (prdIndex - csmIndex >= 1)
            ;
        if (a.getFrame(src))
        {
        }
        else
        {
            cout << "getFrame failed!\n";
            continue;
        }
        ++prdIndex;
    }
}

void mainPC::ImageConsumer()
{
    list<int> xList;
    auto aim_categories = aim.add_array("categories", 20);
    auto aim_data = aim.add_array("delta_x", 20);
    auto aim_fixdata = aim.add_array("fix_delta_x", 20);

    auto chassis_categories = chassis.add_array("categories", 20);
    auto chassis_worldx_data = chassis.add_array("world_delta_x", 20);
    auto chassis_worldy_data = chassis.add_array("world_delta_y", 20);

    cv::dnn::Net net = cv::dnn::readNetFromDarknet("/home/domino/robocon/asset/yolov4-tiny-obj-rc.cfg",
                                                   //"/home/domino/robocon/asset/7.2/yolov4-tiny-obj-rc_best.weights");
                                                   "/home/domino/robocon/asset/6.24/yolov4-tiny-obj-rc_2000.weights");
    net.setPreferableBackend(cv::dnn::Backend::DNN_BACKEND_DEFAULT);
    net.setPreferableTarget(cv::dnn::Target::DNN_TARGET_CPU);
    std::vector<String> outNames = net.getUnconnectedOutLayersNames();
    vector<string> classNamesVec;
    ifstream classNamesFile("/home/domino/robocon/asset/obj-rc.names");
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
    while (1)
    {
        while (prdIndex - csmIndex == 0)
            ;
        data_mt.lock();

        Mat src_copy;
        Rect _dect_rect;
        src.copyTo(fix_img);
        src.copyTo(src_copy);

        int64 start = getTickCount();
        try
        {
            Mat inputBlob = cv::dnn::blobFromImage(src_copy, 1 / 255.F, Size(416, 416), Scalar(), true, false);
            net.setInput(inputBlob);
            // 检测
            std::vector<Mat> outs;
            net.forward(outs, outNames);
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
            int midIndex = -1;

            //autoAim = 3;
            newSerialDataLock.lock();
            autoAim = newSerialData.isAutoAim.d;
            int stm32Time = newSerialData.tr_data_systerm_time.d;
            float world_delta_x = newSerialData.tr_data_act_pos_sys_x.d - newSerialData.world_x.d;
            float world_delta_y = newSerialData.tr_data_act_pos_sys_y.d - newSerialData.world_y.d;
            newSerialDataLock.unlock();

            if (autoAim != 0)
                cout << "autoAim: " << autoAim << endl;
            else
                cout << "autoAim: close\n";

            for (size_t i = 0; i < indices.size(); ++i)
            {
                int idx = indices[i];
                Rect box = boxes[idx];
                int delta_x = abs(getCenterPoint(box).x - 1280 / 2.0);
                char rb;
                char which_one;
                if (autoAim >= 1 && autoAim <= 5)
                    rb = 'r';
                else if (autoAim >= 6 && autoAim <= 10)
                    rb = 'b';
                else
                    rb = ' ';
                int t_autoAim = autoAim;
                if (t_autoAim > 5) //换算
                    t_autoAim -= 5;
                switch (t_autoAim)
                {
                case 1:
                    which_one = '1';
                    break;
                case 2:
                    which_one = '2';
                    break;
                case 3:
                    which_one = '3';
                    break;
                case 4:
                    which_one = '2';
                    break;
                case 5:
                    which_one = '1';
                    break;
                }

                if (delta_x < distance &&
                    classNamesVec[classIds[idx]][0] == rb &&
                    classNamesVec[classIds[idx]][1] == which_one)
                {
                    cout << rb << " " << which_one << endl;
                    distance = delta_x;
                    midBox = box;
                    midIndex = idx;
                    break;
                }
            }
            if (indices.size() != 0 && autoAim != 0 && midIndex != -1)
            {
                Rect big_box(rangeLimitPoint(midBox.tl() - Point(15, 10)),
                             rangeLimitPoint(midBox.br() + Point(15, 10)));
                rectangle(fix_img, midBox, Scalar(0, 0, 255), 2);
                rectangle(fix_img, big_box, Scalar(255, 255, 255), 2);

                vector<vector<Point>> contours;
                vector<Vec4i> hierarchy;
                Mat tmp_src;
                cvtColor(src_copy(big_box), tmp_src, COLOR_BGR2GRAY);

                findContours(tmp_src, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE); //查找轮廓

                Point pt[512];  //存储连通区域个数
                Moments moment; //矩
                int maxArea = -1;
                Point maxCoor;
                for (int i = 0; i < contours.size(); i++) //读取每一个轮廓求取重心
                {

                    double t = contourArea(contours.at(i));
                    if (maxArea < t)
                    {
                        maxArea = t;
                        Mat temp(contours.at(i));
                        moment = moments(temp, false);
                        maxCoor.x = cvRound(moment.m10 / moment.m00);
                        maxCoor.y = cvRound(moment.m01 / moment.m00);
                    }
                }

                Point ans1 = rangeLimitPoint(Point(maxCoor.x + big_box.x, maxCoor.y + big_box.y));
                circle(fix_img, ans1, 3, Scalar(255, 255, 255), 1);
                Point ans = rangeLimitPoint(Point(midBox.x + midBox.width / 2, midBox.y + midBox.height / 2));
                circle(fix_img, ans - Point(0, 30), 3, Scalar(255, 255, 0), 1);
                String className = "total:" + to_string(indices.size()) + "  " + classNamesVec[classIds[midIndex]];
                putText(fix_img, className.c_str(), midBox.tl(), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 0, 0), 2, 8);

                Send2Stm32 send_data{int16_t(ans.x - 1280 / 2.0), float(0)};
                unsigned char Tdata[9];
                Tdata[0] = 0xA5;
                Tdata[1] = 0x5A;
                Tdata[2] = send_data.delta_x_pixel.c[0];
                Tdata[3] = send_data.delta_x_pixel.c[1];
                Tdata[4] = send_data.delta_angle_yaw.c[0];
                Tdata[5] = send_data.delta_angle_yaw.c[1];
                Tdata[6] = send_data.delta_angle_yaw.c[2];
                Tdata[7] = send_data.delta_angle_yaw.c[3];
                Append_CRC8_Check_Sum(Tdata, 9);
                wzSerialportPlus.send(Tdata, 9);

                last_delta_x = send_data.delta_x_pixel.d;

                while (xList.size() > 10)
                    xList.pop_front();
                xList.push_back(last_delta_x);
                /*
                int avg = 0;
                for(int i = xList.begin(); i< xList.end(); ++i){
                    avg += xList[i];
                }
                avg /= xList.size();
                if (abs(avg - xList.back()) < 150)
                {

                }
                */
                aim_categories.push_back(now::ms());
                if (int(last_delta_x) > 35)
                    aim_data.push_back(35);
                else if (int(last_delta_x) < -35)
                    aim_data.push_back(-35);
                else
                    aim_data.push_back(int(last_delta_x));
                int t_x = last_delta_x;
                kal.kalmanFilter(t_x);
                aim_fixdata.push_back(t_x);
            }

            chassis_categories.push_back(now::ms());
            if (int(world_delta_x) > 200 || int(world_delta_y) > 200)
            {
                if (int(world_delta_x) > 200)
                    chassis_worldx_data.push_back(200);
                else if (int(world_delta_y) > 200)
                    chassis_worldy_data.push_back(200);
            }
            else
            {
                chassis_worldx_data.push_back(int(world_delta_x));
                chassis_worldy_data.push_back(int(world_delta_y));
            }

            float fps = getTickFrequency() / (getTickCount() - start);
            ostringstream ss;
            ss << "FPS : " << fps;
            line(fix_img, Point2i(1280 / 2, 0), Point2i(1280 / 2, 1080), cv::Scalar(255, 0, 0), 2);
            putText(fix_img, ss.str(), Point(20, 40), 0, 1, Scalar(0, 0, 255), 2);
            putText(fix_img, "last_delta_x: " + to_string(last_delta_x), Point(20, 100), 0, 1, Scalar(0, 0, 255), 2);
        }
        catch (cv::Exception e)
        {
            cerr << e.what() << endl;
        }

        data_mt.unlock();

        ++csmIndex;
    }
}
