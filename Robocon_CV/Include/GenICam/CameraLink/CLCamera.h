#ifndef __DAHUA_GENICAM_ICLCAMERA_H__
#define __DAHUA_GENICAM_ICLCAMERA_H__

#include "GenICam/Defs.h"
#include "GenICam/Camera.h"
#include "Memory/SharedPtr.h"

GENICAM_NAMESPACE_BEGIN

class ICLCamera;
typedef Memory::TSharedPtr<ICLCamera> ICLCameraPtr;

/// \~chinese
/// \brief 相机对象接口类
/// \defgroup CLCamera  CameraLink相机对象操作接口
/// \~english
/// \brief	camera object interface class
/// \defgroup CLCamera API for CameraLink Cameras Only
/// @{

/// \~chinese
/// \brief Class CameraLink相机对象
/// \~english
/// \brief Class CL camera object
class GENICAM_API ICLCamera
{
protected:
    /// \~chinese
    /// \brief 析构函数
    /// \~english
    /// \brief destruct function 
    virtual ~ICLCamera(){}

public:
    /// \~chinese
    /// \brief CameraLink相机对象获取接口，同一个cameraPtr对应的是同一个CameraLink相机对象
    /// \param [in] cameraPtr CL类型的相机智能指针对象,如果传入了其它非CameraLink相机类型，该接口返回空指针,表示无效
    /// \~english
    /// \brief CL camera object access interface, same cameraPtr is correspond to same CL camera object
    /// \brief [in] cameraPtr Smart pointer object of CL camera, if the cameraPtr point to a camera which is not CL camera, the interface return NULL which means it is invalid
    static  ICLCameraPtr getInstance(const ICameraPtr &cameraPtr);

    /// \~chinese
    /// \brief 获取相机的供应商名
    /// \return 返回相机的供应商名，失败返回NULL
    /// \~english
    /// \brief	get vendor name of camera
    /// \return success:return vendor name of camera, fail:return NULL
	virtual const char * getVenderName() = 0;

    /// \~chinese
    /// \brief 获取相机的型号
    /// \return 返回相机的型号，失败返回NULL
    /// \~english
    /// \brief	get model name of camera
    /// \return success:return camera's model name, fail:return NULL
	virtual const char * getModelName() = 0;

    /// \~chinese
    /// \brief 返回相机的制造商名
    /// \return 返回相机的制造商名，失败返回NULL
    /// \~english
    /// \brief  get manufactureInfo of camera
    /// \return success:return camera's manufactureInfo, fail:return NULL
	virtual const char * getManufactureInfo() = 0;

    /// \~chinese
    /// \brief 获取相机的固件版本
    /// \return 返回相机的固件版本，失败返回NULL
    /// \~english
    /// \brief  get Device Version of camera
    /// \return success:return camera's version, fail:return NULL
	virtual const char * getVersion() = 0;

    /// \~chinese
    /// \brief 获取相机的序列号
    /// \return 返回相机的序列号，失败返回NULL
    /// \~english
    /// \brief	get serial number of camera
    /// \return success:return camera's serial number, fail:return NULL
	virtual const char * getSerialNum() = 0;
};

/// @}

GENICAM_NAMESPACE_END

#endif//__DAHUA_GENICAM_ICLCAMERA_H__