#pragma once
#include <cmath>
using namespace std;

class kalman
{
public:
    float x_last = 0; //上一次的最优值
    float p_last = 0; //上一次最优值的偏差
    float Q = 5.0;  //自己预估的不确定度 越大越代表信任测量值 越小越代表信任预测值
    float R = 5.0;  //测量偏差
    float kg;         //协方差
    float x_mid;      //上次最优值
    float p_mid;      //预估偏差
    float p_now;      //本次最优值

    void kalmanFilter(int &x_now);
};