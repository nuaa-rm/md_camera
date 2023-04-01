//
// Created by bismarck on 23-4-1.
//

#include <fstream>
#include "md_camera/config.h"
#include "md_camera/cameraMatrix.h"
#include "md_camera/resolution.h"


Config::Config() {
    isInit = false;
    yamlNode = YAML::LoadFile(CONFIG_PATH);
    autoExp = yamlNode["AutoExp"].as<bool>();
    expTime = yamlNode["ExpTime"].as<int>();
    resolution = yamlNode["Resolution"].as<std::string>();
    gain = yamlNode["Gain"].as<double>();
    autoWB = yamlNode["AutoWB"].as<bool>();
}

Config::~Config() {
    delete server;
}

void Config::expCallback(const std_msgs::Int32ConstPtr& msg) {
    if (msg->data != expTime && msg->data > 500 && msg->data < 10000) {
        expTime = msg->data;
        camera->lock();
        camera->SetExposureTime(false, expTime);
        camera->unlock();
        internalConfig.ExpTime = expTime;
        server->updateConfig(internalConfig);
    }
}

void Config::gainCallback(const std_msgs::Float64ConstPtr& msg) {
    if (msg->data != gain && msg->data > 10 && msg->data < 300) {
        gain = msg->data;
        camera->lock();
        camera->SetGain(gain);
        camera->unlock();
        internalConfig.Gain = gain;
        server->updateConfig(internalConfig);
    }
}

void Config::resolutionCallback(const std_msgs::StringConstPtr& msg) {
    if (msg->data != resolution) {
        resolution = msg->data;
        camera->lock();
        camera->SetResolution(resolution);
        camera->unlock();
        internalConfig.Resolution = resolution;
        server->updateConfig(internalConfig);
        updateCamInfo(camInfo, resolutionStructCreator(resolution));
    }
}

void Config::configCallback(md_camera::CameraConfig &_config, uint32_t _) {
    bool expTimeModified = _config.ExpTime != expTime;
    if (expTimeModified) {
        expTime = _config.ExpTime;
    }
    if (_config.AutoExp != autoExp || !isInit) {
        autoExp = _config.AutoExp;
        camera->lock();
        camera->SetExposureTime(autoExp, expTime);
        camera->unlock();
    } else {
        if (!autoExp && expTimeModified) {
            camera->lock();
            camera->SetExposureTime(false, expTime);
            camera->unlock();
        }
    }

    if (_config.Resolution != resolution || !isInit) {
        resolution = _config.Resolution;
        camera->lock();
        camera->SetResolution(resolution);
        camera->unlock();
        updateCamInfo(camInfo, resolutionStructCreator(resolution));
    }

    if (_config.Gain != gain || !isInit) {
        gain = _config.Gain;
        camera->lock();
        camera->SetGain(gain);
        camera->unlock();
    }

    if (_config.AutoWB == autoWB || !isInit) {
        autoWB = _config.AutoWB;
        camera->lock();
        camera->SetWBMode(autoWB);
        camera->unlock();
    }

    if (_config.CameraName != internalConfig.CameraName) {
        camera->lock();
        camera->SetCameraName(_config.CameraName);
        camera->unlock();
    }

    if (_config.SetOnceWB) {
        camera->lock();
        camera->SetOnceWB();
        camera->unlock();
        _config.SetOnceWB = false;
        server->updateConfig(_config);
    }

    if (_config.Save) {
        save();
        _config.Save = false;
        server->updateConfig(_config);
    }

    internalConfig = _config;
}

void Config::cameraPubWorker() {
    ros::Rate loopRate(5);
    while(ros::ok()) {
        std_msgs::String msg;
        msg.data = internalConfig.CameraName;
        cameraNamePub.publish(msg);
        cameraInfoPub.publish(camInfo);
        loopRate.sleep();
    }
}

bool Config::setCameraInfoCallback(sensor_msgs::SetCameraInfo::Request &req,
                                   sensor_msgs::SetCameraInfo::Response &res)
{
    camInfo = req.camera_info;
    CameraMatrix info = ci2cm(req.camera_info);
    camera->lock();
    int status = camera->SetCameraMatrix(info);
    camera->unlock();
    res.success = status == 0;
    res.status_message = "Already write to camera!";
    return true;
}

void Config::save() {
    yamlNode["AutoExp"] = autoExp;
    yamlNode["ExpTime"] = expTime;
    yamlNode["Resolution"] = resolution;
    yamlNode["Gain"] = gain;
    yamlNode["AutoWB"] = autoWB;
    std::ofstream file(CONFIG_PATH);
    file << yamlNode;
}

void Config::init(MDCamera *_camera) {
    camera = _camera;
    ros::NodeHandle nh("~");
    camInfo = cm2ci(camera->GetCameraMatrix(), resolutionStructCreator(resolution));
    server = new dynamic_reconfigure::Server<md_camera::CameraConfig>;
    expSub = nh.subscribe("exposure", 1, &Config::expCallback, this);
    gainSub = nh.subscribe("gain", 1, &Config::gainCallback, this);
    resolutionSub = nh.subscribe("resolution", 1, &Config::resolutionCallback, this);
    cameraInfoService = nh.advertiseService("set_camera_info", &Config::setCameraInfoCallback, this);
    cameraNamePub = nh.advertise<std_msgs::String>("camera_name", 1);
    cameraInfoPub = nh.advertise<std_msgs::String>("camera_info", 1);
    md_camera::CameraConfig initConfig;
    initConfig.AutoExp = autoExp;
    initConfig.ExpTime = expTime;
    initConfig.Resolution = resolution;
    initConfig.Gain = gain;
    initConfig.AutoWB = autoWB;
    initConfig.SetOnceWB = false;
    initConfig.Save = false;
    initConfig.CameraName = camera->GetCameraName();
    server->updateConfig(initConfig);
    server->setCallback(boost::bind(&Config::configCallback, this, _1, _2));

    if (yamlNode["HWTrigger"].as<bool>()) {
        camera->SetTriggerMode(MDCamera::hardware);
    } else {
        camera->SetTriggerMode(MDCamera::continuous);
    }

    isInit = true;
    cameraPubThread = std::thread(&Config::cameraPubWorker, this);
}



