#include <opencv2/highgui.hpp>
#include <thread>

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
TCPServer tcpp;
WzSerialportPlus wzSerialportPlus;

extern mutex data_mt;
extern Mat src, fix_img;
extern Json r;
extern int autoAim;

void sig_exit(int s)
{
    tcpp.closed();
    wzSerialportPlus.close();
    exit(0);
}

void tcpTask()
{
    vector<int> opts = {SO_REUSEPORT, SO_REUSEADDR};
    if (tcpp.setup(6666, opts) == 0)
    {
        while (1)
        {
            tcpp.accepted();
            cerr << "Accepted" << endl;
        }
    }
}

void serialTask()
{

    wzSerialportPlus.setReceiveCalback([&](unsigned char *data, int length)
                                       {
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
                                               int8_t(0), int8_t(0), float(0), float(0), float(0),
                                               int8_t(0), int8_t(0), int8_t(0)};

                                           memcpy(&send_data, data, sizeof(send_data));
                                           if (send_data.tr_data_systerm_time.d != old_t)
                                           {
                                               lock_guard<mutex> guard(tcpp.mtSerialData);
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

              cv::Mat tmp_fiximg = cv::imread("/home/domino/OpenCV_web_test/open.jpg");
              cv::Mat tmp_src = cv::imread("/home/domino/OpenCV_web_test/open.jpg");
              std::vector<uchar> buf;

              while (true)
              {
                  data_mt.lock();
                  fix_img.copyTo(tmp_fiximg);
                  data_mt.unlock();
                  if (tmp_fiximg.empty() || tmp_fiximg.channels() != 3)
                      continue;
                  resize(tmp_fiximg, tmp_fiximg, Size(1280/2, 1080/2), 0, 0);
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
        .get("/", [](auto, auto res)
             {
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

    http::Server s;
    s.on_req(
        [](const http::Req &req, http::Res &res)
        {
            if (req.is_method_get())
            {
                if (req.url() == "/json")
                {
                    res.set_status(200);
                    if(autoAim != 0)
                    {
                        data_mt.lock();
                        Json tmp_r;
                        tmp_r = r;
                        r["categories"].set_array();
                        r["data"].set_array();
                        
                        data_mt.unlock();
                        res.set_body(tmp_r.str().c_str());
                    }
                    
                }
                else
                {
                    res.set_status(404);
                }
            }
            else
            {
                res.set_status(405);
            }
        });
    s.start("0.0.0.0", 80);

    tcpp.join();
    seriall.join();
    img2web.join();
    pro.join();
    con.join();

    return 0;
}
