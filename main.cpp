#include <opencv2/highgui.hpp>

#include "mainPC.h"
#include "server.h"
#include "WzSerialportPlus.h"
#include "CRC_Check.h"
#include "ShareMemory.h"

SendData newSerialData{
    int8_t(1), int32_t(0),
    int16_t(0), int16_t(0), int16_t(0), int16_t(0),
    int32_t(0), int32_t(0), int32_t(0), int32_t(0),
    int16_t(0), int8_t(0),

    int8_t(0), int16_t(0), int16_t(0), int16_t(0),
    float(0), int32_t(0), int32_t(0), int32_t(0),
    float(0), float(0), int8_t(0),

    int8_t(0), float(0),
    int8_t(0), int8_t(0), float(0), float(0), float(0), uint16_t(0), float(0), float(0),
    int8_t(0), int8_t(0), int8_t(0)};
vector<SendData> toJson;
WzSerialportPlus wzSerialportPlus;
mutex toJsonLock, newSerialDataLock;

extern mutex data_mt;
extern Mat src, fix_img, res1;
extern Json aim, chassis, excel;
extern int autoAim;

void sig_exit(int s)
{
    wzSerialportPlus.close();
    exit(0);
}

std::string readFileIntoString(char *filename)
{
    std::ifstream ifile(filename);
    std::ostringstream buf;
    char ch;
    while (buf && ifile.get(ch))
        buf.put(ch);
    return buf.str();
}

void serialTask()
{
    wzSerialportPlus.setReceiveCalback(
        [&](unsigned char *data, int length)
        {
            int32_t old_t;
            SendData send_data{
                int8_t(1), int32_t(0),
                int16_t(0), int16_t(0), int16_t(0), int16_t(0),
                int32_t(0), int32_t(0), int32_t(0), int32_t(0),
                int16_t(0), int8_t(0),

                int8_t(0), int16_t(0), int16_t(0), int16_t(0),
                float(0), int32_t(0), int32_t(0), int32_t(0),
                float(0), float(0), int8_t(0),

                int8_t(0), float(0),
                int8_t(0), int8_t(0), float(0), float(0), float(0), uint16_t(0), float(0), float(0),
                int8_t(0), int8_t(0), int8_t(0)};

            memcpy(&send_data, data, sizeof(send_data));
            if (send_data.tr_data_systerm_time.d != old_t)
            {
                lock_guard<mutex> guard(toJsonLock);
                toJson.push_back(send_data);
                newSerialDataLock.lock();
                newSerialData = send_data;
                newSerialDataLock.unlock();
            }
            old_t = send_data.tr_data_systerm_time.d;
        });
    wzSerialportPlus.open("", 115200, 1, 8, 'n');
}

void img2WebTask()
{
    uint16_t port = 5555; /// Set the port
    httpp::Server s(port);

    s.get("/img", [&](auto, auto res)
          {
              res.headers.push_back("Connection: close");
              res.headers.push_back("Max-Age: 0");
              res.headers.push_back("Expires: 0");
              res.headers.push_back("Cache-Control: no-cache, private");
              res.headers.push_back("Pragma: no-cache");
              res.headers.push_back("Content-Type: multipart/x-mixed-replace;boundary=--boundary");

              if (!res.send_header())
                  return;

              cv::Mat tmp_fiximg = cv::imread("/home/domino/robocon/open.jpg");
              cv::Mat tmp_src = cv::imread("/home/domino/robocon/open.jpg");
              std::vector<uchar> buf;

              while (true)
              {
                  
                  data_mt.lock();
                  fix_img.copyTo(tmp_fiximg);
                  src.copyTo(tmp_src);
                  data_mt.unlock();
                  if (tmp_fiximg.empty() || tmp_fiximg.channels() != 3)
                      continue;
                  resize(tmp_fiximg, tmp_fiximg, Size(1280 / 2, 1080 / 2), 0, 0);
                  cv::imencode(".jpg", tmp_fiximg, buf);
                  std::string image(buf.begin(), buf.end());
                  if (!res.send_msg("--boundary\r\n"
                                    "Content-Type: image/jpeg\r\n"
                                    "Content-Length: " +
                                    std::to_string(image.size()) +
                                    "\r\n\r\n" +
                                    image))
                      return;
              }
          })
        .get("/aim", [](auto, auto res)
             {
                 std::string str;
                 str = readFileIntoString("/home/domino/robocon/aim.html");
                 res >> str;
             })
        .get("/chassis", [](auto, auto res)
             {
                 std::string str;
                 str = readFileIntoString("/home/domino/robocon/chassis.html");
                 res >> str;
             })
        .get("/", [](auto, auto res)
             {
                 std::string str;
                 str = readFileIntoString("/home/domino/robocon/img.html");
                 res >> str;
             })
        .listen();
}

int main()
{
    signal(SIGINT, sig_exit);

    thread seriall(serialTask);
    thread img2web(img2WebTask);

    int setting;
    mainPC image_cons_prod(&setting);
    thread pro(&mainPC::ImageProducer, std::ref(image_cons_prod)); // pass by reference
    thread con(&mainPC::ImageConsumer, std::ref(image_cons_prod));

    http::Server s;
    s.on_req(
        [](const http::Req &req, http::Res &res)
        {
            if (req.is_method_get())
            {
                if (req.url() == "/aim.json")
                {

                    res.set_status(200);
                    res.add_header("Access-Control-Allow-Origin", "*");
                    if (autoAim != 0)
                    {
                        data_mt.lock();
                        Json tmp_r;
                        tmp_r = aim;
                        aim["categories"].set_array();
                        aim["delta_x"].set_array();
                        aim["fix_delta_x"].set_array();
                        aim["laser_dis"].set_array();
                        data_mt.unlock();
                        res.set_body(tmp_r.str().c_str());
                    }
                }
                else if (req.url() == "/chassis.json")
                {
                    res.set_status(200);
                    res.add_header("Access-Control-Allow-Origin", "*");
                    data_mt.lock();
                    Json tmp_r;
                    tmp_r = chassis;
                    chassis["categories"].set_array();
                    chassis["world_delta_x"].set_array();
                    chassis["world_delta_y"].set_array();
                    chassis["world_x"].set_array();
                    chassis["world_y"].set_array();
                    chassis["act_x"].set_array();
                    chassis["act_y"].set_array();
                    data_mt.unlock();
                    res.set_body(tmp_r.str().c_str());
                }
                else if (req.url() == "/excel.json")
                {
                    res.set_status(200);
                    res.add_header("Access-Control-Allow-Origin", "*");
                    data_mt.lock();
                    Json tmp_r;
                    tmp_r = excel;
                    excel["data"]["laser_dis"] = 0;
                    excel["data"]["aim_tract"] = 0;
                    excel["data"]["pitch_angle"] = 0;
                    data_mt.unlock();
                    res.set_body(tmp_r.str().c_str());
                }
                else
                    res.set_status(404);
            }
            else
                res.set_status(405);
        });
    s.start("0.0.0.0", 80);

    seriall.join();
    img2web.join();
    pro.join();
    con.join();

    return 0;
}
