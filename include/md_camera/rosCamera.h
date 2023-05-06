//
// Created by bismarck on 23-4-1.
//

#ifndef SRC_ROSCAMERA_H
#define SRC_ROSCAMERA_H

#include <string>
#include <thread>

#include <yaml-cpp/yaml.h>
#include <boost/lockfree/spsc_queue.hpp>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/SetCameraInfo.h>
#include "md_camera/GetCameraInfo.h"
#include "md_camera/CameraConfig.h"

#include "md_camera/MDCamera.h"
#include "md_camera/Recorder.h"

typedef boost::lockfree::spsc_queue<LockFrame*> FrameQueue;


class RosCamera {
private:
    YAML::Node yamlNode;

    bool isInit = false, isRecord = false;
    int fpsLimit = 0, recordFps = 0;
    double hwInterval = 0;
    std::shared_ptr<MDCamera> camera = std::make_shared<MDCamera>();
    md_camera::CameraConfig internalConfig;
    sensor_msgs::CameraInfo camInfo;
    std::string frame_id;

    dynamic_reconfigure::Server<md_camera::CameraConfig> *server = nullptr;
    ros::Subscriber expSub;
    ros::Subscriber gainSub;
    ros::Subscriber resolutionSub;
    ros::Subscriber recordSub;
    ros::Publisher cameraNamePub;
    ros::Publisher cameraInfoPub;
    ros::Publisher imagePub;
    ros::ServiceServer setCameraInfoService;
    ros::ServiceServer getCameraInfoService;

    std::thread cameraPubThread;
    std::thread imageGetThread;
    std::thread imagePubThread;
    std::thread imageRecordThread;

    FrameQueue publishQueue{10};
    FrameQueue recordQueue{10};

    Recorder recorder{RECORD_PATH};

    void cameraPubWorker();
    void getImageWorker();
    void publishImageWorker();
    void recordImageWorker();

    void startRecord(const std::string& resolution);
    void stopRecord();

    void expCallback(const std_msgs::Int32ConstPtr& msg);
    void gainCallback(const std_msgs::Float64ConstPtr& msg);
    void resolutionCallback(const std_msgs::StringConstPtr& msg);
    void recordCallback(const std_msgs::BoolConstPtr& msg);
    void configCallback(md_camera::CameraConfig& _config, uint32_t _);
    bool setCameraInfoCallback(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &res);
    bool getCameraInfoCallback(md_camera::GetCameraInfo::Request &_, md_camera::GetCameraInfo::Response &res);
public:
    void init();
    void save();

    ~RosCamera();
};


#endif //SRC_ROSCAMERA_H
