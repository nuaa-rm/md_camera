/// Please see camera development documentation.

#include <CameraDefine.h>
#include <ros/ros.h>

#include "md_camera/resolution.h"
#include "md_camera/MDCamera.h"
#include "md_camera/check.h"

INIT_CHECK_RETRY()

int MDCamera::Init(const std::string& camera_name) {
    printf("CAMERA SDK INIT...\n");
    CHECK_ABORT_RETRY(3, CameraSdkInit(1));
    printf("DONE\n");


    printf("ENUM CAMERA DEVICES...\n");
    //枚举设备，并建立设备列表
    tSdkCameraDevInfo tCameraEnumList[MAX_CAMERA_NUM];
    CHECK_ABORT(CameraEnumerateDevice(tCameraEnumList, &iCameraCounts));
    //没有连接设备
    if (iCameraCounts == 0) {
        printf("ERROR: NO CAMERA CONNECTED.\n");
        return -1;
    } else {
        printf("CONNECTED CAMERA NUMBERS: %d.\n", iCameraCounts);
    }
    printf("DONE\n");

    int id = 0;
    if (!camera_name.empty()) {
        id = -1;
        for (int i = 0; i < iCameraCounts; i++) {
            std::string camName = string(tCameraEnumList[i].acFriendlyName);
            if (camName == camera_name) {
                id = i;
                break;
            }
        }
        if (id == -1) {
            std::cerr << "CANNOT FIND CAMERA NAMED " << camera_name << std::endl;
            abort();
        }
    }

    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    CHECK_ABORT_RETRY(3, CameraInit(&(tCameraEnumList[id]), -1, -1, &hCamera));

    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CHECK_ABORT_RETRY(3, CameraGetCapability(hCamera, &mCapability));
    CHECK_ABORT_RETRY(3, CameraPlay(hCamera));

    //设置输出为彩色
    CHECK_ABORT_RETRY(3, CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8));
    CHECK_ABORT_RETRY(3, CameraSetFrameSpeed(hCamera, 1));
    printf("CAMERA INIT SUCCESS.\n");

    started = true;
    return 0;
}

int MDCamera::Uninit() {
    if (started) {
        printf("Uninit...\n");
        CHECK_RETURN_RETRY(CameraUnInit(hCamera), 3);
        printf("CAMERA UNINIT SUCCESS!\n");
    }
    started = false;
}

int MDCamera::SetExposureTime(double exp_time) {
    CHECK_RETURN_RETRY(3, CameraSetExposureTime(hCamera, exp_time));
    printf("SET EXP TIME SUCCESS.\n");
    CHECK_RETURN(CameraGetCapability(hCamera, &mCapability));

    return 0;
}

int MDCamera::SetExposureMode(bool auto_exp) {
    CHECK_RETURN_RETRY(3, CameraSetAeState(hCamera, auto_exp));
    printf("ENABLE AUTO EXP SUCCESS.\n");
    CHECK_RETURN(CameraGetCapability(hCamera, &mCapability));
    return 0;
}

int MDCamera::SetResolution(const std::string &idx) {
    if (idx.size() > 31) {
        std::cerr << "Resolution Index TOO LONG, Max size: 31" << std::endl;
        return -1;
    }
    tSdkImageResolution sRoiResolution = resolutionStructCreator(idx);

    CHECK_RETURN_RETRY(3, CameraSetImageResolution(hCamera, &sRoiResolution));
    printf("CAMERA SET RESOLUTION SUCCESS.\n");

    resolution_idx = idx;

    postProcessBuffers.resetSlotSize(mCapability.sResolutionRange.iHeightMax
                                        * mCapability.sResolutionRange.iWidthMax * 3);

    CHECK_RETURN_RETRY(3, CameraGetCapability(hCamera, &mCapability));

    return 0;
}

// white balance
int MDCamera::SetWBMode(bool auto_wb) const {
    CHECK_RETURN_RETRY(3, CameraSetWbMode(hCamera, auto_wb));
    printf("CAMERA SETWBMODE %d SUCCESS!\n", auto_wb);
    return 0;
}

int MDCamera::SetOnceWB() const {
    CHECK_RETURN_RETRY(3, CameraSetOnceWB(hCamera));
    printf("CAMERA SETONCEWB SUCCESS!\n");
    return 0;
}

int MDCamera::SetGain(double gain) {
    CHECK_RETURN_RETRY(3, CameraSetGain(hCamera, gain, gain, gain));
    printf("CAMERA SETGAIN SUCCESS!\n");
    CHECK_RETURN(CameraGetCapability(hCamera, &mCapability));
    return 0;
}

int MDCamera::LoadParameters(int group) {
    if (group == PARAMETER_TEAM_DEFAULT) {
        resolution_idx = "MaxSize";
    }
    CHECK_RETURN_RETRY(3, CameraLoadParameter(hCamera, group));
    printf("CAMERA SET SATURATION SUCCESS!\n");
    CHECK_RETURN(CameraGetCapability(hCamera, &mCapability));
    CHECK_RETURN(CameraSetRotate(hCamera, 2));
    return 0;
}

int MDCamera::SetTriggerMode(MDCamera::triggerMode mode) const {
    CHECK_RETURN_RETRY(3, CameraSetTriggerMode(hCamera, mode));
    printf("CAMERA SET TRIGGER MODE SUCCESS!\n");
    return 0;
}

int MDCamera::Play() {
    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CHECK_RETURN(CameraGetCapability(hCamera, &mCapability));
    CHECK_RETURN_RETRY(3, CameraPlay(hCamera));
    printf("CAMERA PLAY SUCCESS!\n");
    return 0;
}

///
/// \brief MVCamera::GetFrame_B
/// \param frame
/// true if we want bgr img
/// \return
///
int MDCamera::GetFrame(LockFrame** frame) {
    if (started) {
        auto slot = postProcessBuffers.getSlot();
        if (slot == nullptr) {
            return -1;
        }
        CHECK_RETURN(CameraGetImageBufferPriority(hCamera, slot->headPtr(), &pbyBuffer, 1000, 1));
        CHECK_RETURN(CameraImageProcess(hCamera, pbyBuffer, slot->data(), slot->headPtr()));
        CHECK_RETURN(CameraReleaseImageBuffer(hCamera, pbyBuffer));
        *frame = slot;
    }
    return 0;
}

std::string MDCamera::GetCameraName() const {
    char name[32] = {0};
    CHECK(CameraGetFriendlyName(hCamera, name));
    return {name};
}

int MDCamera::SetCameraName(std::string _name) const {
    if (_name.size() > 31) {
        return -1;
    }
    char name[33] = {0};
    strcpy(name, _name.data());
    CHECK_RETURN(CameraSetFriendlyName(hCamera, name));
    return 0;
}

CameraMatrix MDCamera::GetCameraMatrix() const {
    CameraMatrix res{};
    CHECK(CameraReadSN(hCamera, (uint8_t*)&res, 1));
    return res;
}

int MDCamera::SetCameraMatrix(CameraMatrix data) const {
    CHECK_RETURN(CameraWriteSN(hCamera, (uint8_t*)&data, 1));
    return 0;
}

MDCamera::~MDCamera() {
    Uninit();
}

MDCamera &MDCamera::operator=(MDCamera &&other) noexcept {
    if (&other == this) {
        return *this;
    }

    mtx = other.mtx;
    hCamera = other.hCamera;
    iCameraCounts = other.iCameraCounts;
    channel = other.channel;
    mCapability = other.mCapability;
    pbyBuffer = other.pbyBuffer;
    postProcessBuffers = std::move(other.postProcessBuffers);
    started = other.started;

    other.hCamera = 0;
    other.started = false;
    other.pbyBuffer = nullptr;
    other.mtx = make_shared<std::mutex>();

    return *this;
}

void MDCamera::lock() {
    mtx->lock();
}

void MDCamera::unlock() {
    mtx->unlock();
}
