#include "kalman.h"

void kalman::kalmanFilter(int &x_now)
{
    x_mid = x_last;                       //上次最优值
    p_mid = p_last + Q;                   //预估偏差
    kg = sqrt(p_mid / (p_mid + R));       //协方差
    x_now = x_mid + kg * (x_now - x_mid); //最优值
    p_now = 1 - kg * p_mid;               //最优偏差
    x_last = x_now;                       //保留最优值
    p_last = p_now;                       //保留最优偏差
}