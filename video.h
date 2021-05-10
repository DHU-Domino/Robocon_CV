#pragma once

#include "GxIAPI.h"
#include "DxImageProc.h"

#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cstdio>

#define BYTE unsigned char

using namespace std;
using namespace cv;

static void GX_STDC OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame);

class video {
public:
    video();
    ~video();

    void initParam();
    int videoCheck();
    bool videoOpen();
    bool getFrame(Mat& img);
    bool streamControl(int n=1);

private:
    GX_STATUS status = GX_STATUS_SUCCESS;
    GX_DEV_HANDLE hDevice = NULL;
    GX_OPEN_PARAM stOpenParam;
    uint32_t nDeviceNum = 0;

    static BYTE         *m_pBufferRaw;          ///< 原始图像数据
    static BYTE         *m_pBufferRGB;          ///< RGB图像数据，用于显示和保存bmp图像
    static int64_t      m_nImageHeight;         ///< 原始图像高
    static int64_t      m_nImageWidth;          ///< 原始图像宽
    static int64_t      m_nPayLoadSize;
    static int64_t      m_nPixelColorFilter;    ///< Bayer格式
    static int64_t      m_nPixelFormat;
    static Mat img;

    static void GX_STDC OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame);
};
