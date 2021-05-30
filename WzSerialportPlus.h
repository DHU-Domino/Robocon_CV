#pragma once

#include <iostream>
#include <string>
#include <cstring>
#include <thread>
#include <functional>
#include <ctime>
#include <cstdio>
#include <cstdlib>

#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <getopt.h>

#pragma pack(1)

//字节数为4的uchar数据类型
typedef union
{
    int32_t d;
    unsigned char c[4];
} int32uchar;

//字节数为4的uchar数据类型
typedef union
{
    float d;
    unsigned char c[4];
} float2uchar;

//字节数为2的uchar数据类型
typedef union
{
    int16_t d;
    unsigned char c[2];
} int16uchar;

//字节数为1的uchar数据类型
typedef union
{
    int8_t d;
    unsigned char c;
} int8uchar;

//用于保存
typedef struct
{
    int8uchar tr_data_sof1; //0xA5
    int32uchar tr_data_systerm_time;
    int16uchar tr_data_chassis_speed_rpm1;
    int16uchar tr_data_chassis_speed_rpm2;
    int16uchar tr_data_chassis_speed_rpm3;
    int16uchar tr_data_chassis_speed_rpm4;
    int32uchar tr_data_chassis_total_ecd1;
    int32uchar tr_data_chassis_total_ecd2;
    int32uchar tr_data_chassis_total_ecd3;
    int32uchar tr_data_chassis_total_ecd4; //29
    int16uchar tr_data_trace_speed_rpm;    //31
    int8uchar crc8_0;                      //32

    int8uchar tr_data_sof2; //0x5A
    int16uchar tr_data_pitch_speed_rpm;
    int16uchar tr_data_load_l_speed_rpm;
    int16uchar tr_data_load_r_speed_rpm;
    int32uchar tr_data_traction_total_ecd;
    int32uchar tr_data_pitch_total_ecd;
    int32uchar tr_data_load_l_total_ecd;
    int32uchar tr_data_load_r_total_ecd;
    float2uchar tr_data_act_pos_sys_x; //27
    float2uchar tr_data_act_pos_sys_y; //31
    int8uchar crc8_1;                  //32

    int8uchar tr_data_sof3;            //0x55
    float2uchar tr_data_act_pos_sys_w; //5

    int8uchar isAutoAim;         //6
    int8uchar aim_which_bucket;  //7
    float2uchar world_x;         //11
    float2uchar world_y;         //15
    float2uchar world_angle_yaw; //19

    int8uchar crc8_2; //20
    int8uchar ret;    //21
    int8uchar enter;  //22

} SendData;

typedef struct
{
    //int8uchar tr_data_sof1;      //0xA5
    //int8uchar tr_data_sof2;      //0x5A
    int16uchar delta_x_pixel;    //偏移的像素，参考
    float2uchar delta_angle_yaw; //偏移的角度
    //int8uchar crc8;              //3
} Send2Stm32;

using ReceiveCallback = std::function<void(unsigned char *, int)>;

class WzSerialportPlus
{
public:
    WzSerialportPlus();
    /**
     * @param name: serialport name , such as: /dev/ttyS1
     * @param baudrate: baud rate , valid: 2400,4800,9600,19200,38400,57600,115200,230400
     * @param stopbit: stop bit , valid: 1,2
     * @param databit: data bit , valid: 7,8
     * @param paritybit: parity bit , valid: 'o'/'O' for odd,'e'/'E' for even,'n'/'N' for none
     */
    WzSerialportPlus(const std::string &name,
                     const int &baudrate,
                     const int &stopbit,
                     const int &databit,
                     const int &paritybit);
    ~WzSerialportPlus();

    bool open();

    /**
     * @brief parameters same as constructor
     */
    bool open(const std::string &name,
              const int &baudrate,
              const int &stopbit,
              const int &databit,
              const int &paritybit);

    void close();

    int send(unsigned char *data, int length);

    void setReceiveCalback(ReceiveCallback receiveCallback);

protected:
    virtual void onReceive(unsigned char *data, int length);

private:
    int serialportFd;
    std::string name;
    int baudrate;
    int stopbit;
    int databit;
    int paritybit;
    bool receivable;
    int receiveMaxlength;
    int receiveTimeout; /* unit: ms */
    ReceiveCallback receiveCallback;
};