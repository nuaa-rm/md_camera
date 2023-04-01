
#ifndef MVCAMERA_MVCAMERA_H
#define MVCAMERA_MVCAMERA_H

#include <mutex>
#include <CameraApi.h>  //相机SDK头文件
#include <opencv2/opencv.hpp>
#include <md_camera/cameraMatrix.h>

#define MAX_CAMERA_NUM 4

using namespace std;
using namespace cv;

class MDCamera {
private:
    CameraHandle hCamera = 0;
    int iCameraCounts = MAX_CAMERA_NUM;
    int channel = 3;
    tSdkCameraCapbility mCapability{};                      //设备描述信息
    tSdkFrameHead mFrameInfo{};
    unsigned char *pbyBuffer = nullptr;
    unsigned char *g_pRgbBuffer[2]{nullptr, nullptr};       //处理后数据缓存区
    bool started = false;
    std::string resolution_idx{};
    std::shared_ptr<std::mutex> mtx = make_shared<std::mutex>();

    void freeBuffer();
public:
    enum triggerMode {
        continuous = 0,
        hardware = 2
    };

    int Init(const std::string& camera_name="");
    void Uninit();

    int Play();

    int LoadParameters(int group=PARAMETER_TEAM_DEFAULT);

    int SetExposureTime(bool auto_exp, double exp_time=10000);

    int SetResolution(const std::string& idx);

    int SetGain(double gain);

    int SetWBMode(bool auto_wb = true) const;
    int SetOnceWB() const;

    std::string GetCameraName() const;
    int SetCameraName(std::string _name) const;

    CameraMatrix GetCameraMatrix() const;
    int SetCameraMatrix(CameraMatrix data) const;

    int SetTriggerMode(triggerMode mode) const;

    int GetFrame(Mat &frame);

    void lock();
    void unlock();

    MDCamera() = default;
    MDCamera(MDCamera&& other) = default;
    MDCamera(MDCamera& other) = delete;
    MDCamera& operator=(MDCamera&& other) noexcept ;
    ~MDCamera();
};


#endif //MVCAMERA_MVCAMERA_H
