#include "WzSerialportPlus.h"
#include "CRC_Check.h"

using namespace std;

string get_uart_dev_name()
{
    FILE *ls = popen("ls /dev/ttyUSB* --color=never", "r");
    char name[20] = {0};
    if (fscanf(ls, "%s", name))
    {
    }
    return name;
}

WzSerialportPlus::WzSerialportPlus()
    : serialportFd(-1),
      name(""),
      baudrate(115200),
      stopbit(1),
      databit(8),
      paritybit('n'),
      receivable(false),
      receiveMaxlength(128),
      receiveTimeout(5000),
      receiveCallback(nullptr)
{
}

WzSerialportPlus::WzSerialportPlus(const std::string &name,
                                   const int &baudrate,
                                   const int &stopbit,
                                   const int &databit,
                                   const int &paritybit) : serialportFd(-1),
                                                           name(name),
                                                           baudrate(baudrate),
                                                           stopbit(stopbit),
                                                           databit(databit),
                                                           paritybit(paritybit),
                                                           receivable(false),
                                                           receiveMaxlength(128),
                                                           receiveTimeout(5000),
                                                           receiveCallback(nullptr)
{
}

WzSerialportPlus::~WzSerialportPlus()
{
    close();

    while (receivable)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

bool WzSerialportPlus::open()
{
    serialportFd = ::open(name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serialportFd < 0)
    {
        return false;
    }

    if (fcntl(serialportFd, F_SETFL, 0) < 0)
    {
        return false;
    }

    if (isatty(serialportFd) == 0)
    {
        ::close(serialportFd);
        return false;
    }

    struct termios newtio, oldtio;
    if (tcgetattr(serialportFd, &oldtio) != 0)
    {
        return false;
    }

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;
    switch (databit)
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    default:
        return false;
    }
    switch (paritybit)
    {
    case 'o':
    case 'O':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'e':
    case 'E':
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'n':
    case 'N':
        newtio.c_cflag &= ~PARENB;
        break;
    default:
        return false;
    }
    switch (stopbit)
    {
    case 1:
        newtio.c_cflag &= ~CSTOPB;
        break;
    case 2:
        newtio.c_cflag |= CSTOPB;
        break;
    default:
        return false;
    }
    switch (baudrate)
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 19200:
        cfsetispeed(&newtio, B19200);
        cfsetospeed(&newtio, B19200);
        break;
    case 38400:
        cfsetispeed(&newtio, B38400);
        cfsetospeed(&newtio, B38400);
        break;
    case 57600:
        cfsetispeed(&newtio, B57600);
        cfsetospeed(&newtio, B57600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    case 230400:
        cfsetispeed(&newtio, B230400);
        cfsetospeed(&newtio, B230400);
        break;
    default:
        return false;
    }

    newtio.c_cc[VTIME] = 1;
    newtio.c_cc[VMIN] = 1;

    tcflush(serialportFd, TCIFLUSH);
    if (tcsetattr(serialportFd, TCSANOW, &newtio) != 0)
    {
        return false;
    }

    tcflush(serialportFd, TCIOFLUSH);
    fcntl(serialportFd, F_SETFL, 0);

    receivable = true;

    std::thread([&] {
        unsigned char *receiveData = new unsigned char[receiveMaxlength];
        int receivedLength = 0;

        unsigned char rightData[100];
        int rightLength = 0;
        int cntResult = 0;

        memset(rightData, 0, sizeof(rightData));
        while (receivable)
        {
            memset(receiveData, 0, receiveMaxlength);

            receivedLength = read(serialportFd, receiveData, receiveMaxlength);
            if (receivedLength > 0)
            {
                if (receivedLength == 32 && receiveData[0] == 0xA5)
                { //第一段
                    if (Verify_CRC8_Check_Sum(receiveData, 32))
                    {
                        for (int i = 0; i < 32; ++i)
                        {
                            rightData[i] = receiveData[i];
                        }
                    }
                    rightLength += receivedLength;
                    ++cntResult;
                }
                else if (receivedLength == 32 && receiveData[0] == 0x5A)
                {
                    if (Verify_CRC8_Check_Sum(receiveData, 32))
                    {
                        for (int i = 32; i < 64; ++i)
                        {
                            rightData[i] = receiveData[i - 32];
                        }
                    }
                    rightLength += receivedLength;
                    ++cntResult;
                }
                else if (receiveData[0] == 0x55)
                {
                    if (Verify_CRC8_Check_Sum(receiveData, 26))
                    {
                        for (int i = 64; i < receivedLength + 64; ++i)
                        {
                            rightData[i] = receiveData[i - 64];
                        }
                    }
                    rightLength += receivedLength;
                    ++cntResult;
                }
                onReceive(receiveData, receivedLength);
                if (nullptr != receiveCallback && cntResult == 3)
                {
                    if (rightLength == 92)
                    {
                        receiveCallback(rightData, rightLength);
                    }
                    cntResult = 0;
                    rightLength = 0;
                    memset(rightData, 0, sizeof(rightData));
                }
            }
            receivedLength = 0;
        }

        delete[] receiveData;
        receiveData = nullptr;
    }).detach();
    return true;
}

bool WzSerialportPlus::open(const std::string &name,
                            const int &baudrate,
                            const int &stopbit,
                            const int &databit,
                            const int &paritybit)
{
    if (name == "")
    {
        this->name = get_uart_dev_name();
    }
    else
    {
        this->name = name;
    }
    this->baudrate = baudrate;
    this->stopbit = stopbit;
    this->databit = databit;
    this->paritybit = paritybit;
    return open();
}

void WzSerialportPlus::close()
{
    if (receivable)
    {
        receivable = false;
    }

    if (serialportFd >= 0)
    {
        ::close(serialportFd);
        serialportFd = -1;
    }
}

int WzSerialportPlus::send(unsigned char *data, int length)
{
    int lengthSent = 0;
    lengthSent = write(serialportFd, data, length);
    return lengthSent;
}

void WzSerialportPlus::setReceiveCalback(ReceiveCallback receiveCallback)
{
    this->receiveCallback = receiveCallback;
}

void WzSerialportPlus::onReceive(unsigned char *data, int length)
{
}