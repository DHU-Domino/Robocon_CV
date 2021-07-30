#include "mainPC.h"

#include "WzSerialportPlus.h"
#include "CRC_Check.h"

using namespace std;
using namespace cv;

std::mutex data_mt;
Mat src, fix_img;
Json aim, chassis, excel;
int autoAim = 0, aimPosi = 1;

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
    FileStorage storage("/home/domino/robocon/setting.xml", FileStorage::READ);
    color = storage["Color"].real();
    target_control = storage["Target"].real();
    position_control = storage["Position"].real();
    deltaX[0][0] = storage["P1D1"].real();
    deltaX[0][1] = storage["P1D2"].real();
    deltaX[0][2] = storage["P1D3"].real();
    deltaX[0][3] = storage["P1D4"].real();
    deltaX[0][4] = storage["P1D5"].real();

    deltaX[1][0] = storage["P2D1"].real();
    deltaX[1][1] = storage["P2D2"].real();
    deltaX[1][2] = storage["P2D3"].real();
    deltaX[1][3] = storage["P2D4"].real();
    deltaX[1][4] = storage["P2D5"].real();

    deltaX[2][0] = storage["P3D1"].real();
    deltaX[2][1] = storage["P3D2"].real();
    deltaX[2][2] = storage["P3D3"].real();
    deltaX[2][3] = storage["P3D4"].real();
    deltaX[2][4] = storage["P3D5"].real();

    saveDebug = storage["saveDebug"].real();
    saveRecord = storage["saveRecord"].real();
    ExposeTime = storage["ExposeTime"].real();

    if(1 == saveRecord)
    {
        video_writer_src = initVideoWriter("_src");
        video_writer_fix = initVideoWriter("_fix");
    }
        
}

VideoWriter mainPC::initVideoWriter(string x) {
    VideoWriter video;
    std::ifstream in("/home/domino/robocon/record/cnt.txt");
    int cnt = 0;
    if (in.is_open()) {
        in >> cnt;
        in.close();
    }
    std::string file_name = "/home/domino/robocon/record/" + std::to_string(cnt) + x + ".avi";
    cnt++;
    if(x == "_fix")
    {
        std::ofstream out("/home/domino/robocon/record/cnt.txt");
        if (out.is_open()) {
            out << cnt << std::endl;
            out.close();
        }
    }
    video.open(file_name, VideoWriter::fourcc('M', 'J', 'P', 'G'), 20, cv::Size(1280, 1024), true);
    return video;
}

void mainPC::switchModel(int color)
{
    if (0 == color)
    {
        net = cv::dnn::readNetFromDarknet("/home/domino/robocon/asset/yolov4-tiny-obj-rc-class_red_3.cfg",
                                          //"/home/domino/robocon/asset/7.23-d2-red3/yolov4-tiny-obj-rc-class_red_3_final.weights");
                                          "/home/domino/robocon/asset/7.26-d2-red3/yolov4-tiny-obj-rc-class_red_3_final.weights");
        classNamesFile.open("/home/domino/robocon/asset/obj-rc-class_red_3.names");
    }
    else if (1 == color)
    {
        net = cv::dnn::readNetFromDarknet("/home/domino/robocon/asset/yolov4-tiny-obj-rc-class_blue_3.cfg",
                                          //"/home/domino/robocon/asset/7.23-d2-blue3/yolov4-tiny-obj-rc-class_blue_3_final.weights");
                                          "/home/domino/robocon/asset/7.26-d2-blue3/yolov4-tiny-obj-rc-class_blue_3_final.weights");
        classNamesFile.open("/home/domino/robocon/asset/obj-rc-class_blue_3.names");
    }
    else if (2 == color)
    {
        net = cv::dnn::readNetFromDarknet("/home/domino/robocon/asset/yolov4-tiny-obj-rc.cfg",

                                          //"/home/domino/robocon/asset/6.22/yolov4-tiny-obj-rc_final.weights");
                                          //"/home/domino/robocon/asset/7.15/yolov4-tiny-obj-rc_final.weights");
                                          //"/home/domino/robocon/asset/7.15/continue/yolov4-tiny-obj-rc_best.weights");
                                          //"/home/domino/robocon/asset/7.17/yolov4-tiny-obj-rc_final.weights");
                                          //"/home/domino/robocon/asset/7.22-d2-1/yolov4-tiny-obj-rc_final.weights");
                                          //"/home/domino/robocon/asset/7.22-d2-2/yolov4-tiny-obj-rc_final.weights");
                                          "/home/domino/robocon/asset/7.23-d2-1/yolov4-tiny-obj-rc_final.weights");
        classNamesFile.open("/home/domino/robocon/asset/obj-rc.names");
    }
    net.setPreferableBackend(cv::dnn::Backend::DNN_BACKEND_DEFAULT);
    net.setPreferableTarget(cv::dnn::Target::DNN_TARGET_CPU);

    outNames.clear();
    classNamesVec.clear();

    outNames = net.getUnconnectedOutLayersNames();
    if (classNamesFile.is_open())
    {
        string className = "";
        while (std::getline(classNamesFile, className))
        {
            cerr << className << '\n';
            classNamesVec.push_back(className);
        }
        classNamesFile.close();
    }
}
void mainPC::ImageProducer()
{
    if (!a.videoCheck())
    {
        cerr << "没相机\n";
    }
    if (a.videoOpen())
    {
        a.initParam(ExposeTime);
        a.streamControl(1);
        cerr << "ok\n";
    }
    
    while (1)
    {
        while (prdIndex - csmIndex >= 1)
            ;
        if (a.getFrame(src))
        {
            if (src.empty())
                continue;
            if (3 != src.channels())
                continue;
        }
        else
        {
            cerr << "getFrame failed!\n";
            continue;
        }
        ++prdIndex;
    }
}

Point getMoments(Mat t, Rect r)
{
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    Mat tmp_src;
    cvtColor(t(r), tmp_src, COLOR_BGR2GRAY);
    findContours(tmp_src, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE); //查找轮廓

    Point pt[512];  //存储连通区域个数
    Moments moment; //矩
    int maxArea = -1;
    Point maxCoor;
    for (int i = 0; i < contours.size(); ++i) //读取每一个轮廓求取重心
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
    return rangeLimitPoint(Point(maxCoor.x + r.x, maxCoor.y + r.y));
}

void mainPC::ImageConsumer()
{
    list<int> xList;
    auto aim_categories = aim.add_array("categories", 20);
    auto aim_data = aim.add_array("delta_x", 20);
    auto aim_fixdata = aim.add_array("fix_delta_x", 20);
    auto aim_laser_dis = aim.add_array("laser_dis", 20);
    auto o = excel.add_object("data", 2);
    o.add_member("laser_dis", 0);
    o.add_member("aim_tract", 0);
    o.add_member("pitch_angle", 0);

    auto chassis_categories = chassis.add_array("categories", 20);
    auto chassis_worldx_data = chassis.add_array("world_delta_x", 20);
    auto chassis_worldy_data = chassis.add_array("world_delta_y", 20);

    auto chassis_act_x = chassis.add_array("act_x", 20);
    auto chassis_act_y = chassis.add_array("act_y", 20);

    auto chassis_world_x = chassis.add_array("world_x", 20);
    auto chassis_world_y = chassis.add_array("world_y", 20);

    switchModel(color);

    int last_delta_x = 0;
    int aim_flag = 0;
    int old_t;
    while (1)
    {
        while (prdIndex - csmIndex == 0)
            ;
        int64 start = getTickCount();
        data_mt.lock();

        Mat src_copy;
        src.copyTo(fix_img);
        src.copyTo(src_copy);
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
            cv::dnn::NMSBoxes(boxes, confidences, 0.1, 0.3, indices);

            newSerialDataLock.lock();
            if (target_control > 0)
                autoAim = target_control;
            else
                autoAim = newSerialData.isAutoAim.d;
            if (position_control > 0)
                aimPosi = position_control;
            else
                aimPosi = aimPosi = newSerialData.aim_posi.d;
            int stm32Time = newSerialData.tr_data_systerm_time.d;
            //cerr << "stm32Time: " << stm32Time << endl;
            float act_x = newSerialData.tr_data_act_pos_sys_x.d;
            float act_y = newSerialData.tr_data_act_pos_sys_y.d;
            float world_x = newSerialData.world_x.d;
            float world_y = newSerialData.world_y.d;
            float aim_tract = newSerialData.aim_tract.d;
            float pitch_angle = newSerialData.pitch_angle.d;
            float avg_laser_dis = newSerialData.avg_laser_dis.d;

            newSerialDataLock.unlock();

            float world_delta_x = act_x - world_x;
            float world_delta_y = act_y - world_y;
            char rb, which_one = '0';
            if (autoAim != 0)
            {
                aim_flag = 1;
                excel["data"]["laser_dis"] = 0;
                excel["data"]["aim_tract"] = 0; //有自瞄的时候都是0
                excel["data"]["pitch_angle"] = 0;
                //cerr << "autoAim: " << autoAim << '\n';
                if (autoAim >= 1 && autoAim <= 5)
                {
                    rb = 'r';
                    if(1 == color)
                    {
                        color = RED;
                        switchModel(color);
                    }
                }
                    
                else if (autoAim >= 6 && autoAim <= 10)
                {
                    rb = 'b';
                    if(0 == color)
                    {
                        color = BLUE;
                        switchModel(color);
                    } 
                }
                else
                    rb = ' ';

                int t_autoAim = autoAim;
                if (t_autoAim > 5) //换算
                    t_autoAim -= 5;
                if (1 == t_autoAim || 5 == t_autoAim)
                    which_one = '1';
                else if (2 == t_autoAim || 4 == t_autoAim)
                    which_one = '2';
                else
                    which_one = '3';
            }
            else
            {
                if (1 == aim_flag) //autoAim的一次下降沿
                {
                    aim_flag = 0;
                    excel["data"]["laser_dis"] = avg_laser_dis; //没自瞄的时候都是这个数
                    excel["data"]["aim_tract"] = aim_tract;
                    excel["data"]["pitch_angle"] = pitch_angle;
                    cerr << "autoAim: close\n";
                }
                
            }

            vector<detectAns> dAns;
            int dAnsCnt = 0;

            for (size_t i = 0; i < indices.size(); ++i)
            {
                int idx = indices[i];
                Rect box = boxes[idx];
                int delta_x = abs(getCenterPoint(box).x - 1280 / 2.0);

                if (classNamesVec[classIds[idx]][0] == rb &&
                    classNamesVec[classIds[idx]][1] == which_one) //tmd这个whichone是字符，不是数字
                {
                    cerr << rb << which_one << '\n';
                    struct detectAns temp_ans;
                    temp_ans.distance = delta_x;
                    temp_ans.midBox = box;
                    temp_ans.midIndex = idx;
                    dAns.push_back(temp_ans);
                    ++dAnsCnt;
                }
                rectangle(fix_img, box, Scalar(0, 0, 255), 1);
                putText(fix_img, classNamesVec[classIds[idx]].c_str(), box.tl(), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 0, 0), 2, 8);
            }
            if (saveDebug == 1)
            {
                int t = chrono::duration_cast<chrono::seconds>(chrono::system_clock::now().time_since_epoch()).count();
                if (t - old_t > 10)
                {
                    string s = "/home/domino/Picture/" + to_string(t);
                    cv::imwrite(s + "src.bmp", src);
                    cv::imwrite(s + "fix_img.bmp", fix_img);
                    old_t = t;
                }
            }

            if (0 != indices.size() && 0 != autoAim && 0 != dAnsCnt)
            {
                if (aim_tract != 0 && avg_laser_dis != 0)
                    cerr << "avg_laser_dis: " << avg_laser_dis << "\taim_tract: " << aim_tract << endl;

                struct detectAns goodAns;
                if ((2 <= dAnsCnt) && ('1' == which_one || '2' == which_one)) // 有两解且可能出现两解
                {
                    for (int i = 0; i < dAnsCnt - 1; ++i) //简单的冒泡，甚至连剪枝都不写
                        for (int j = 0; j < dAnsCnt - 1; ++j)
                            if (dAns[j].midBox.x > dAns[j + 1].midBox.x)
                            {
                                detectAns temp = dAns[j];
                                dAns[j] = dAns[j + 1];
                                dAns[j + 1] = temp;
                            }
                    detectAns l_ans = dAns[0], r_ans = dAns[dAnsCnt - 1];
                    if (autoAim >= 6)
                        autoAim -= 5;
                    if (which_one == '2') //只有二型桶涉及左右
                    {
                        if (autoAim - (aimPosi - 1) * 5 == 2) // 2号桶
                        {
                            if (aimPosi == 1)
                                goodAns = r_ans;
                            else
                                goodAns = l_ans;
                        }
                        else // 4号桶
                        {
                            if (aimPosi == 1)
                                goodAns = l_ans;
                            else
                                goodAns = r_ans;
                        }
                    }
                    else if (which_one == '1') //1左5右
                        if (autoAim == 1)      // 1号桶
                            goodAns = l_ans;
                        else // 5号桶
                            goodAns = r_ans;
                }
                else
                    goodAns = dAns[0];

                Rect big_box(rangeLimitPoint(goodAns.midBox.tl() - Point(15, 10)),
                             rangeLimitPoint(goodAns.midBox.br() + Point(15, 10)));
                Point momentsPoint = getMoments(src_copy, big_box);
                Point ans = rangeLimitPoint(Point(goodAns.midBox.x + goodAns.midBox.width / 2,
                                                  goodAns.midBox.y + goodAns.midBox.height / 2));
                rectangle(fix_img, goodAns.midBox, Scalar(0, 0, 255), 2);
                rectangle(fix_img, big_box, Scalar(255, 255, 255), 2);
                circle(fix_img, momentsPoint, 3, Scalar(255, 255, 255), 1);
                circle(fix_img, ans - Point(0, 30), 3, Scalar(255, 255, 0), 1);

                if (autoAim >= 6)
                    autoAim -= 5;
                last_delta_x = ans.x - 1280 / 2.0 + deltaX[aimPosi - 1][autoAim - 1];

                while (xList.size() >= 4)
                    xList.pop_front();
                xList.push_back(last_delta_x); //保持列表长度为10

                int avg = 0;
                for (int n : xList)
                    avg += n;
                avg = avg / (int)xList.size(); //对4个数求平均
                if (abs(avg - xList.back()) > 150)
                {
                    xList.pop_back();
                    xList.push_back(avg);
                }
                last_delta_x = xList.back();

                Send2Stm32 send_data{int16_t(last_delta_x), float(1)};
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

                aim_categories.push_back(now::ms());
                if (avg_laser_dis > 1000)
                    aim_laser_dis.push_back(1000);
                else
                    aim_laser_dis.push_back((int)avg_laser_dis);

                if (int(last_delta_x) > 35)
                    aim_data.push_back(35);
                else if (int(last_delta_x) < -35)
                    aim_data.push_back(-35);
                else
                    aim_data.push_back(int(last_delta_x));
                //int t_x = last_delta_x;
                //kal.kalmanFilter(t_x);
                //aim_fixdata.push_back(t_x);
            }

            chassis_categories.push_back(now::ms());

            if (int(world_delta_x) > 200)
                chassis_worldx_data.push_back(200);
            else
                chassis_worldx_data.push_back(int(world_delta_x));
            if (int(world_delta_y) > 200)
                chassis_worldy_data.push_back(200);
            else
                chassis_worldy_data.push_back(int(world_delta_y));

            chassis_act_x.push_back(int(act_x));
            chassis_act_y.push_back(int(act_y));
            chassis_world_x.push_back(int(world_x));
            chassis_world_y.push_back(int(world_y));

            float fps = getTickFrequency() / (getTickCount() - start);
            ostringstream ss;
            ss << "FPS : " << fps;
            line(fix_img, Point2i(1280 / 2, 0), Point2i(1280 / 2, 1080), cv::Scalar(255, 0, 0), 1);
            putText(fix_img, ss.str(), Point(20, 40), 0, 1, Scalar(0, 0, 255), 2);
            putText(fix_img, "last_delta_x: " + to_string(last_delta_x), Point(20, 100), 0, 1, Scalar(0, 0, 255), 2);
            if(1 == saveRecord)
            {
                video_writer_src.write(src);
                video_writer_fix.write(fix_img);
            }
        }
        catch (cv::Exception e)
        {
            cerr << e.what() << '\n';
        }

        data_mt.unlock();

        ++csmIndex;
    }
}
