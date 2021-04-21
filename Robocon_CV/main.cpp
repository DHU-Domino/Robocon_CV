#include "ImageConsProd.hpp"
#include <conio.h>
#include <thread>

using namespace cv;
using namespace std;

TCPServer tcp;
int sid = -1;
bool isRecord = false;

void createTCPServer() {
    vector<int> opts = { SO_REUSEADDR };
    if (tcp.setup(6666, opts) == 0) {
        while (1) {
            sid = tcp.accepted();
            cerr << "Accepted" << endl;
        }
    }
    else {
        cerr << "Errore apertura socket" << endl;
    }
}

void readKeyboard() {
    int ch;
    while (1) {
        if (_kbhit()) {         //如果有按键按下，则_kbhit()函数返回真
            ch = _getch();      //使用_getch()函数获取按下的键值
            //cout << ch;
            if (-1 != sid) {
                tcp.Send(string(1, ch), sid);
                cout << string(1, ch);
                if ("k" == string(1, ch)) {
                    isRecord = true;
                }
                else if ("l" == string(1, ch)) {
                    isRecord = false;
                }
            }
            if (ch == 27) {     //当按下ESC时循环，ESC键的键值时27.
                break;
            }
        }
    }
}

int main()
{
    int a = 1;
    ImageConsProd image_cons_prod(a);

    std::thread t1(&ImageConsProd::ImageProducer, std::ref(image_cons_prod)); // pass by reference
    std::thread t2(&ImageConsProd::ImageConsumer, std::ref(image_cons_prod));
    //std::thread tcp(&createTCPServer);
    //std::thread keyboard(&readKeyboard);

    t1.join(); t2.join(); //tcp.join(); keyboard.join();
}
