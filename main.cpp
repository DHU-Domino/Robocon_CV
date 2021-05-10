#include <opencv2/highgui.hpp>
#include <thread>
#include <fstream>

#include "server.h"

#include "WzSerialportPlus.h"
#include "TCPServer.h"
#include "CRC_Check.h"
#include "mainPC.h"

#include <sys/types.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>

ProdCons_data pc_data[1];
TCPServer tcp;
WzSerialportPlus wzSerialportPlus;

extern mutex data_mt;
extern Mat img;

void sig_exit(int s)
{
    tcp.closed();
    wzSerialportPlus.close();
    exit(0);
}

void tcpTask()
{
    vector<int> opts = {SO_REUSEPORT, SO_REUSEADDR};
    if (tcp.setup(6666, opts) == 0)
    {
        while (1)
        {
            tcp.accepted();
            cerr << "Accepted" << endl;
        }
    }
}

void serialTask()
{

    wzSerialportPlus.setReceiveCalback([&](unsigned char *data, int length) {
        int32_t old_t;
        SendData send_data{
            int8_t(1), int32_t(0),
            int16_t(0), int16_t(0), int16_t(0), int16_t(0),
            int32_t(0), int32_t(0), int32_t(0), int32_t(0),
            int16_t(0), int8_t(0),

            int8_t(0), int16_t(0), int16_t(0), int16_t(0),
            int32_t(0), int32_t(0), int32_t(0), int32_t(0),
            float(0), float(0), int8_t(0),

            int8_t(0), float(0),
            int8_t(0), int8_t(0), int8_t(0)};

        memcpy(&send_data, data, sizeof(send_data));
        if (send_data.tr_data_systerm_time.d != old_t)
        {
            lock_guard<mutex> guard(tcp.mtSerialData);
            pc_data[0].data = send_data;
            pc_data[0].isUse = 0;
        }
        old_t = send_data.tr_data_systerm_time.d;
    });
    wzSerialportPlus.open("", 115200, 1, 8, 'n');
}

void img2WebTask()
{
    uint16_t port = 5555; /// Set the port
    http::Server s(port);

    s.get("/img", [&](auto, auto res) {
         res.headers.push_back("Connection: close");
         res.headers.push_back("Max-Age: 0");
         res.headers.push_back("Expires: 0");
         res.headers.push_back("Cache-Control: no-cache, private");
         res.headers.push_back("Pragma: no-cache");
         res.headers.push_back("Content-Type: multipart/x-mixed-replace;boundary=--boundary");

         if (!res.send_header())
             return;

         cv::Mat tmp_data = cv::imread("/home/domino/OpenCV_web_test/open.jpg");
         std::vector<uchar> buf;

         cv::dnn::Net net = cv::dnn::readNetFromDarknet("/home/domino/OpenCV_web_test/asset/yolov4-tiny-obj-rc.cfg",
                                                        "/home/domino/OpenCV_web_test/asset/yolov4-tiny-obj-rc_2000.weights");
         net.setPreferableBackend(cv::dnn::Backend::DNN_BACKEND_OPENCV);
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
                 classNamesVec.push_back(className);
         }
         while (true)
         {
             data_mt.lock();
             img.copyTo(tmp_data);
             data_mt.unlock();
             if (tmp_data.empty() || tmp_data.channels() != 3)
             {
                 continue;
             }

             //resize(tmp_data, tmp_data, Size(750, 600), 0, 0);
             try
             {

                 Mat inputBlob = cv::dnn::blobFromImage(tmp_data, 1 / 255.F, Size(416, 416), Scalar(), true, false);
                 net.setInput(inputBlob);
                 // 检测
                 cerr << "1" << endl;
                 std::vector<Mat> outs;
                 net.forward(outs, outNames);
                 cerr << "outs.size(): " << outs.size() << endl;
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
                             int centerX = (int)(data[0] * tmp_data.cols);
                             int centerY = (int)(data[1] * tmp_data.rows);
                             int width = (int)(data[2] * tmp_data.cols);
                             int height = (int)(data[3] * tmp_data.rows);
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
                 /*
                 for (size_t i = 0; i < indices.size(); ++i)
                 {
                     int idx = indices[i];
                     Rect box = boxes[idx];
                     String className = classNamesVec[classIds[idx]];
                     //putText(tmp_data, className.c_str(), box.tl(), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 0, 0), 2, 8);
                     rectangle(tmp_data, box, Scalar(0, 0, 255), 2, 8, 0);
                 }
                 */
             }
             catch (cv::Exception e)
             {
                 cerr << e.what();
             }

             cv::imencode(".jpg", tmp_data, buf);
             std::string image(buf.begin(), buf.end());
             if (!res.send_msg("--boundary\r\n"
                               "Content-Type: image/jpeg\r\n"
                               "Content-Length: " +
                               std::to_string(image.size()) +
                               "\r\n\r\n" +
                               image))
                 return;
         }
     }).get("/", [](auto, auto res) {
           res >> "<html>"
                  "    <body>"
                  "        <h1>CAMERA STREAMING</h1>"
                  /// Set the correct ip address
                  "        <img src='http://192.168.31.80:5555/img'/>"
                  "    </body>"
                  "</html>";
       })
        .listen();
}

int main()
{
    signal(SIGINT, sig_exit);

    thread tcpp(tcpTask);
    thread seriall(serialTask);
    thread img2web(img2WebTask);

    int setting; //先随便写个int，之后应该改为专门的配置类
    mainPC image_cons_prod(&setting);
    thread pro(&mainPC::ImageProducer, std::ref(image_cons_prod)); // pass by reference
    thread con(&mainPC::ImageConsumer, std::ref(image_cons_prod));

    tcpp.join();
    seriall.join();
    img2web.join();
    pro.join();
    con.join();

    return 0;
}
