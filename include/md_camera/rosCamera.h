//
// Created by bismarck on 23-4-1.
//

#ifndef SRC_ROSCAMERA_H
#define SRC_ROSCAMERA_H

#include <string>
#include <thread>

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/SetCameraInfo.h>
#include "md_camera/CameraConfig.h"

#include "md_camera/MDCamera.h"


class RosCamera {
private:
    YAML::Node yamlNode;

    bool isInit;
    std::shared_ptr<MDCamera> camera = std::make_shared<MDCamera>();
    md_camera::CameraConfig internalConfig;
    sensor_msgs::CameraInfo camInfo;

    dynamic_reconfigure::Server<md_camera::CameraConfig> *server = nullptr;
    ros::Subscriber expSub;
    ros::Subscriber gainSub;
    ros::Subscriber resolutionSub;
    ros::Publisher cameraNamePub;
    ros::Publisher cameraInfoPub;
    ros::Publisher imagePub;
    ros::ServiceServer cameraInfoService;
    std::thread cameraPubThread;
    std::thread imagePubThread;

    void cameraPubWorker();
    void getImageWorker();
public:
    void expCallback(const std_msgs::Int32ConstPtr& msg);
    void gainCallback(const std_msgs::Float64ConstPtr& msg);
    void resolutionCallback(const std_msgs::StringConstPtr& msg);
    void configCallback(md_camera::CameraConfig& _config, uint32_t _);
    bool setCameraInfoCallback(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &res);

    void init();
    void save();

    RosCamera();
    ~RosCamera();
};


#endif //SRC_ROSCAMERA_H
