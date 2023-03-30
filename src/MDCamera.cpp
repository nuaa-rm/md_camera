/// Please see camera development documentation.

#include <CameraDefine.h>
#include <ros/ros.h>

#include "md_camera/resolution.h"
#include "md_camera/MDCamera.h"

struct md_camera_exception : public exception {
    int status;

    explicit md_camera_exception(int _status) {
        status = _status;
    }

    const char *what() const noexcept override {
        return "MDCamera failure";
    }
};

#define CHECK(status) \
    do\
    {\
        auto ret = (status);\
        if (ret != 0)\
        {\
            std::cerr << "MDCamera failure: " << ret << std::endl; \
            throw md_camera_exception(ret);\
        }\
    } while (0)

#define CHECK_RETURN(status) \
    do\
    {\
        auto ret = (status);\
        if (ret != 0)\
        {\
            std::cerr << "MDCamera failure: " << ret << std::endl;\
            return -1;\
        }\
    } while (0)

#define CHECK_ABORT(status) \
    do\
    {\
        auto ret = (status);\
        if (ret != 0)\
        {\
            std::cerr << "MDCamera failure: " << ret << std::endl;\
            abort();\
        }\
    } while (0)

int MDCamera::Init(const std::string& camera_name) {
    printf("CAMERA SDK INIT...\n");
    CHECK_ABORT(CameraSdkInit(1));
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
        for (int i = 0; i < iCameraCounts; i++) {
            std::string camName = string(tCameraEnumList[i].acFriendlyName);
            if (camName == camera_name) {
                id = i;
                break;
            }
        }
    }

    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    CHECK_ABORT(CameraInit(&(tCameraEnumList[id]), -1, -1, &hCamera));
    printf("CAMERA INIT SUCCESS.\n");

    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CHECK_ABORT(CameraGetCapability(hCamera, &mCapability));
    CHECK_ABORT(CameraPlay(hCamera));

    //设置输出为彩色
    CHECK_ABORT(CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8));
    CHECK_ABORT(CameraSetFrameSpeed(hCamera, 1));

    started = true;
    return 0;
}

void MDCamera::Uninit() {
    printf("Save Parameter...\n");
    CHECK_ABORT(CameraSaveParameter(hCamera, 0));

    printf("Uninit...\n");
    CHECK_ABORT(CameraUnInit(hCamera));

    printf("CAMERA UNINIT SUCCESS!\n");

    freeBuffer();
    started = false;
}

int MDCamera::SetExposureTime(bool auto_exp, double exp_time) {
    if (auto_exp) {
        CHECK_RETURN(CameraSetAeState(hCamera, true));
        printf("ENABLE AUTO EXP SUCCESS.\n");
    } else {
        CHECK_RETURN(CameraSetAeState(hCamera, false));
        printf("DISABLE AUTO EXP SUCCESS.\n");
        CHECK_RETURN(CameraSetExposureTime(hCamera, exp_time));
        printf("SET EXP TIME SUCCESS.\n");
    }
    CHECK_RETURN(CameraGetCapability(hCamera, &mCapability));

    return 0;
}

double MDCamera::GetExposureTime() const {
    int auto_exp;
    CHECK(CameraGetAeState(hCamera, &auto_exp));
    if (auto_exp) {
        return 0;
    } else {
        double exp_time;
        CHECK(CameraGetExposureTime(hCamera, &exp_time));
        return exp_time;
    }
}

int MDCamera::SetResolution(const std::string &idx) {
    if (idx.size() > 31) {
        std::cerr << "Resolution Index TOO LONG, Max size: 31" << std::endl;
        return -1;
    }
    tSdkImageResolution sRoiResolution = resolutionStructCreator(idx);

    CHECK_RETURN(CameraSetImageResolution(hCamera, &sRoiResolution));
    printf("CAMERA SET RESOLUTION SUCCESS.\n");

    resolution_idx = idx;

    //初始化缓冲区
    freeBuffer();
    g_pRgbBuffer[0] = (unsigned char *) malloc(
            mCapability.sResolutionRange.iHeightMax * mCapability.sResolutionRange.iWidthMax * 3);
    g_pRgbBuffer[1] = (unsigned char *) malloc(
            mCapability.sResolutionRange.iHeightMax * mCapability.sResolutionRange.iWidthMax * 3);

    CHECK_RETURN(CameraGetCapability(hCamera, &mCapability));

    return 0;
}

// white balance
int MDCamera::SetWBMode(bool auto_wb) const {
    CHECK_RETURN(CameraSetWbMode(hCamera, auto_wb));
    printf("CAMERA SETWBMODE %d SUCCESS!\n", auto_wb);
    return 0;
}

bool MDCamera::GetWBMode() const {
    int res = 0;
    CHECK(CameraGetWbMode(hCamera, &res));
    printf("CAMERA GETWBMODE %d SUCCESS!\n", res);
    return res;
}

int MDCamera::SetOnceWB() const {
    CHECK_RETURN(CameraSetOnceWB(hCamera));
    printf("CAMERA SETONCEWB SUCCESS!\n");
    return 0;
}

int MDCamera::SetGain(double gain) {
    CHECK_RETURN(CameraSetGain(hCamera, gain, gain, gain));
    printf("CAMERA SETGAIN SUCCESS!\n");
    CHECK_RETURN(CameraGetCapability(hCamera, &mCapability));
    return 0;
}

int MDCamera::LoadParameters(int group) {
    if (group == PARAMETER_TEAM_DEFAULT) {
        resolution_idx = "MaxSize";
    }
    CHECK_RETURN(CameraLoadParameter(hCamera, group));
    printf("CAMERA SET SATURATION SUCCESS!\n");
    SetResolution(GetResolution());
    CHECK_RETURN(CameraGetCapability(hCamera, &mCapability));
    return 0;
}

double MDCamera::GetGain() const {
    int r_gain, g_gain, b_gain;
    CHECK(CameraGetGain(hCamera, &r_gain, &g_gain, &b_gain));
    printf("CAMERA GETGAIN SUCCESS!\n");
    return (r_gain + g_gain + b_gain) / 300.;
}

int MDCamera::Play() {
    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CHECK_RETURN(CameraGetCapability(hCamera, &mCapability));
    CHECK_RETURN(CameraPlay(hCamera));
    return 0;
}

///
/// \brief MVCamera::GetFrame_B
/// \param frame
/// true if we want bgr img
/// \return
///
int MDCamera::GetFrame(Mat &frame) {
    if (started) {
        CHECK_RETURN(CameraGetImageBuffer(hCamera, &mFrameInfo, &pbyBuffer, 1000));

        CHECK_RETURN(CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer[0], &mFrameInfo));
        frame = Mat(
                cv::Size(mFrameInfo.iWidth, mFrameInfo.iHeight),
                mFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
                g_pRgbBuffer[0]
        );
        CHECK_RETURN(CameraReleaseImageBuffer(hCamera, pbyBuffer));
    }
    return 0;
}

int MDCamera::Stop() {
    started = false;
    usleep(30000);
    return 0;
}

void MDCamera::freeBuffer() {
    if (g_pRgbBuffer[0] != nullptr) {
        free(g_pRgbBuffer[0]);
        g_pRgbBuffer[0] = nullptr;
    }

    if (g_pRgbBuffer[1] != nullptr) {
        free(g_pRgbBuffer[1]);
        g_pRgbBuffer[1] = nullptr;
    }
}

MDCamera::~MDCamera() {
    Uninit();
}

int MDCamera::SaveParameters(int group) const {
    printf("Save Parameter...\n");
    CHECK_ABORT(CameraSaveParameter(hCamera, group));
    if (!resolution_idx.empty()) {
        char t[32] = {0};
        memcpy(t, resolution_idx.data(), resolution_idx.size());
        CHECK(CameraWriteSN(hCamera, (uint8_t *) t, 1));
    }
    return 0;
}

MDCamera &MDCamera::operator=(MDCamera &&other) noexcept {
    if (&other == this) {
        return *this;
    }
    hCamera = other.hCamera;
    iCameraCounts = other.iCameraCounts;
    channel = other.channel;
    mCapability = other.mCapability;
    mFrameInfo = other.mFrameInfo;
    pbyBuffer = other.pbyBuffer;
    g_pRgbBuffer[0] = other.g_pRgbBuffer[0];
    g_pRgbBuffer[1] = other.g_pRgbBuffer[1];
    started = other.started;

    other.hCamera = 0;
    other.started = false;
    other.pbyBuffer = nullptr;
    other.g_pRgbBuffer[0] = nullptr;
    other.g_pRgbBuffer[1] = nullptr;

    return *this;
}

std::string MDCamera::GetResolution() {
    if (resolution_idx.empty()) {
        char res_idx[32];
        CHECK(CameraReadSN(hCamera, (uint8_t *) res_idx, 1));
        resolution_idx = std::string(res_idx);
    }
    return resolution_idx;
}

int MDCamera::SetTriggerMode(MDCamera::triggerMode mode) const {
    CHECK_RETURN(CameraSetTriggerMode(hCamera, mode));
    return 0;
}

MDCamera::triggerMode MDCamera::GetTriggerMode() const {
    int mode;
    CHECK(CameraGetTriggerMode(hCamera, &mode));
    return (MDCamera::triggerMode)mode;
}
