//
// Created by bismarck on 23-4-1.
//

#ifndef SRC_CONFIG_H
#define SRC_CONFIG_H

#include <string>
#include <thread>

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include "md_camera/CameraConfig.h"

#include "md_camera/MDCamera.h"

class Config {
private:
    YAML::Node yamlNode;
    bool autoExp;
    int expTime;
    std::string resolution;
    double gain;
    bool autoWB;

    bool isInit;
    MDCamera* camera = nullptr;
    md_camera::CameraConfig internalConfig;

    dynamic_reconfigure::Server<md_camera::CameraConfig> *server = nullptr;
    ros::Subscriber expSub;
    ros::Subscriber gainSub;
    ros::Subscriber resolutionSub;
    ros::Publisher cameraNamePub;
    std::thread cameraNameThread;
public:
    void expCallback(const std_msgs::Int32ConstPtr& msg);
    void gainCallback(const std_msgs::Float64ConstPtr& msg);
    void resolutionCallback(const std_msgs::StringConstPtr& msg);
    void configCallback(md_camera::CameraConfig& _config, uint32_t _);
    void cameraNamePubWorker();

    void init(MDCamera* _camera);
    void save();

    Config();
    ~Config();
};


#endif //SRC_CONFIG_H
