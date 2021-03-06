#include <mutex>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "video.h"

using namespace std;

BYTE *video::m_pBufferRaw;
BYTE *video::m_pBufferRGB;
int64_t video::m_nImageHeight;
int64_t video::m_nImageWidth;
int64_t video::m_nPayLoadSize;
int64_t video::m_nPixelColorFilter;
int64_t video::m_nPixelFormat;

cv::Mat video::src_img;
mutex src_imgLock;

video::video()
{
    status = GXInitLib();
    if (status != GX_STATUS_SUCCESS)
        return;
}

video::~video()
{
    status = GXCloseDevice(hDevice);
    status = GXCloseLib();
}
void video::initParam(int ExposeTime)
{
    status = GXSetEnum(hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS); //设置采集模式为连续采集
    status = GXSetEnum(hDevice, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ACQUISITION_FRAME_RATE_MODE_ON);
    status = GXSetFloat(hDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, 80); //设置采集帧率,假设设置为 10.0，用户按照实际需求设置此值
    double dExposureValue = ExposeTime;//rm: day1794 / night?40000   rc: day 11:2500 12:50 6000 / night 18:5000|20:75000
    status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, dExposureValue);
    //status = GXSetEnum(hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
    
    /*//曝光如果合适，自动白平衡也不错
    status = GXSetEnum(hDevice,GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
    status = GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, 1.3398);//rm: day1.5117/night?    rc: day1.3398/night?
    status = GXSetEnum(hDevice,GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
    status = GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, 1.3789);//rm: day1.3672/night?    rc: day1.3789/night?
    */
    status = GXSetEnum(hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
}

int video::videoCheck()//搜索相机
{
    status = GX_STATUS_SUCCESS;
    status = GXUpdateDeviceList(&nDeviceNum, 1000);
    return nDeviceNum <= 0 ? 0 : nDeviceNum;
}
bool video::videoOpen()//初始化相机
{
    stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE;
    stOpenParam.openMode = GX_OPEN_INDEX;
    stOpenParam.pszContent = "1";
    status = GXOpenDevice(&stOpenParam, &hDevice);
    return true;
}

bool video::streamControl(int n)
{
    if (n == 1)
    {
        // 获取图像大小
        status = GXGetInt(hDevice, GX_INT_PAYLOAD_SIZE, &m_nPayLoadSize);
        // 获取宽度
        status = GXGetInt(hDevice, GX_INT_WIDTH, &m_nImageWidth);
        // 获取高度
        status = GXGetInt(hDevice, GX_INT_HEIGHT, &m_nImageHeight);
        src_imgLock.lock();
        src_img.create(m_nImageHeight, m_nImageWidth, CV_8UC3);
        src_imgLock.unlock();
        //判断相机是否支持bayer格式
        bool m_bColorFilter;
        status = GXIsImplemented(hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &m_bColorFilter);
        if (m_bColorFilter)
        {
            status = GXGetEnum(hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &m_nPixelColorFilter);
        }
        GXGetEnum(hDevice, GX_ENUM_PIXEL_FORMAT, &m_nPixelFormat);

        m_pBufferRGB = new BYTE[(size_t)(m_nImageWidth * m_nImageHeight * 3)];
        if (m_pBufferRGB == NULL)
        {
            return false;
        }
        //为存储原始图像数据申请空间
        m_pBufferRaw = new BYTE[(size_t)m_nPayLoadSize];
        if (m_pBufferRaw == NULL)
        {
            delete[] m_pBufferRGB;
            m_pBufferRGB = NULL;
            return false;
        }

        //注册图像处理回调函数
        status = GXRegisterCaptureCallback(hDevice, NULL, OnFrameCallbackFun);
        status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_START);
    }
    else
    {
        status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_STOP);
        status = GXUnregisterCaptureCallback(hDevice);
    }
    return true;
}

bool video::getFrame(cv::Mat &img)
{
    src_imgLock.lock();
    src_img.copyTo(img);
    src_imgLock.unlock();
    if (img.empty() || img.channels() != 3)
        return false;
    return true;
}

void GX_STDC video::OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM *pFrame)
{
    memcpy(m_pBufferRaw, pFrame->pImgBuf, pFrame->nImgSize);
    DxRaw8toRGB24(m_pBufferRaw, m_pBufferRGB, (VxUint32)(m_nImageWidth), (VxUint32)(m_nImageHeight), RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(m_nPixelColorFilter), false);

    src_imgLock.lock();
    memcpy(src_img.data, m_pBufferRGB, m_nImageWidth * m_nImageHeight * 3);
    std::vector<cv::Mat> channels;
    split(src_img, channels);
    std::swap(channels[0], channels[2]);
    merge(channels, src_img);
    src_imgLock.unlock();
}
