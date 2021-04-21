/**
* @file   RecordVideo.h
* @brief  视频录像.
* @author 
* @par    Copyright (c):
*         All Rights Reserved
* @version 1.0.0.0
*/

#ifndef __RECORD_VIDEO_H_
#define __RECORD_VIDEO_H_

#ifdef  __cplusplus
extern "C"{
#endif  // end #ifdef  __cplusplus

#ifdef WIN64
#undef WIN32
#endif

	/** 编译选项 */
#ifdef WIN32  // win64位下，会报和def重复定义的警告 In Win 64bit, it will report warning for duplicate definition
#  ifdef _USRDLL // 动态库导出 Export DLL
#    ifdef RECORDVIDEO_EXPORTS
#		define RECORD_API __declspec(dllexport)
#	 else
#		define RECORD_API __declspec(dllimport)
#	 endif
#  else
#    define RECORD_API
#  endif // end of ifdef _USRDLL
#else
#	define RECORD_API
#endif //end #ifdef WIN32

#if (defined (WIN32) || defined(WIN64))
#   define CALLMETHOD __stdcall
#else
#	define CALLMETHOD
#endif // end #if (defined (WIN32) || defined(WIN64))

    typedef void* HANDLE;        ///< 录像API使用的句柄 Handle for using record API

    /**
    * @enum RECORD_EErr
    * @brief 接口返回值 Return value
    * @attention 无 No
    */
    typedef enum tagRECORD_EErr
    {
        RECORD_SUCCESS = 0,             ///< 成功 Success
        RECORD_ILLEGAL_PARAM,           ///< 非法参数 Illegal parameter
        RECORD_ERR_ORDER,               ///< 调用接口顺序错误 Sequence error for calling interfaces
        RECORD_NO_MEMORY,               ///< 内存不足 No memory
        RECORD_NOT_SUPPORT = 255        ///< 不支持   Not support
    }RECORD_EErr;

    /**
    * @enum RECORD_EVideoFormatType
    * @brief 视频格式 Video format
    * @attention 无 No
    */
    typedef enum tagRECORD_EVideoFormatType
    {
        RECORD_VIDEO_FMT_AVI = 0,               ///< avi格式 format avi
        RECORD_VIDEO_FMT_NOT_SUPPORT = 255      ///< 不支持 Not support
    }RECORD_EVideoFormatType;

    /**
    * @struct RECORD_SRecordParam
    * @brief  打开录像句柄所需要的参数 Parameters needed for opening the record handle
    * @attention 无 No
    */
    typedef struct tagRECORD_SRecordParam
    {
        unsigned int            width;          ///< 图像宽 Image width
        unsigned int            height;         ///< 图像高 Image Height
        float                   frameRate;      ///< 帧率(大于0) frame rate(greater than 0)
        unsigned int            quality;        ///< 视频质量(1-100) video quality(1-100)
        RECORD_EVideoFormatType recordFmtType;  ///< 视频格式 video format
        const char*             recordFilePath;           ///< 保存路径 save path 

        unsigned int            reserved[26];   ///< 预留 Reserved

    }RECORD_SRecordParam;

    /**
    * @struct RECORD_SFrameInfo
    * @brief  录制一帧图像所需要的参数 Parameters needed for recording one frame
    * @attention 无 No
    */
    typedef struct tagRECORD_SFrameInfo
    {
        const unsigned char*    data;           ///< 图像数据指针 Image data pointer
        unsigned int            size;           ///< 图像的长度 Size of image
        unsigned int            paddingX;       ///< 图像宽填充 Padding X
        unsigned int            paddingY;       ///< 图像高填充 Padding Y
        int                     pixelformat;    ///< 图像格式 Image format

        unsigned int            reserved[27];   ///< 预留 Reserved

    }RECORD_SFrameInfo;

    /**
    *  ~chinese
    *  @brief  打开录像
    *  @param[in]  pParam       ：打开录像句柄所需要的参数
    *  @param[out] pHandle      ：录像句柄
    *  @Return:   RECORD_EErr   : 接口返回值
    *  - RECORD_SUCCESS 表示执行成功
    *  - 其他值见RECORD_EErr枚举
    *  ~english
    *  @brief  open record
    *  @param[in]  pParam       ：parameters needed for opening the record handle
    *  @param[out] pHandle      ：record handle	
    *  @Return:   RECORD_EErr   : return value
    *  - RECORD_SUCCESS return ok
    *  - Other values refers to enumeration of RECORD_EErr
    */
    RECORD_API RECORD_EErr CALLMETHOD openRecord(RECORD_SRecordParam *pParam, HANDLE* pHandle);

    /**
    *  ~chinese
    *  @brief  录制一帧图像
    *  @param[in] handle        ：录像句柄
    *  @param[in] pFrameInfo    ：录制一帧图像所需要的参数 
    *  @Return:   RECORD_EErr   : 接口返回值
    *  - RECORD_SUCCESS 表示执行成功
    *  - 其他值见RECORD_EErr枚举
    *  ~english
    *  @brief  record one frame
    *  @param[in] handle        ：record handle
    *  @param[in] pFrameInfo    ：parameters needed for recording one frame
    *  @Return:   RECORD_EErr   : return value
    *  - RECORD_SUCCESS return ok
    *  - Other values refers to enumeration of RECORD_EErr
    */
    RECORD_API RECORD_EErr CALLMETHOD inputOneFrame(HANDLE handle, RECORD_SFrameInfo *pFrameInfo);

    /**
    *  ~chinese
    *  @brief  关闭录像
    *  @param[in] handle        ：录像句柄
    *  @Return:   RECORD_EErr   : 接口返回值
    *  - RECORD_SUCCESS 表示执行成功
    *  - 其他值见RECORD_EErr枚举
    *  ~english
    *  @brief  close record
    *  @param[in] handle        ：record handle
    *  @Return:   RECORD_EErr   : return value
    *  - RECORD_SUCCESS return ok
    *  - Other values refers to enumeration of RECORD_EErr
    */
    RECORD_API RECORD_EErr CALLMETHOD closeRecord(HANDLE handle);

#ifdef  __cplusplus
}
#endif // end #ifdef  __cplusplus

#endif // end of #ifndef __RECORD_VIDEO_H_