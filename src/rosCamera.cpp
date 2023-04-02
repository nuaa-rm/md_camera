//
// Created by bismarck on 23-4-1.
//

#include <fstream>
#include <cv_bridge/cv_bridge.h>
#include "md_camera/rosCamera.h"
#include "md_camera/cameraMatrix.h"
#include "md_camera/resolution.h"


RosCamera::RosCamera() {
    isInit = false;
    yamlNode = YAML::LoadFile(CONFIG_PATH);
    internalConfig.AutoExp = yamlNode["AutoExp"].as<bool>();
    internalConfig.ExpTime = yamlNode["ExpTime"].as<int>();
    internalConfig.Resolution = yamlNode["Resolution"].as<std::string>();
    internalConfig.Gain = yamlNode["Gain"].as<double>();
    internalConfig.AutoWB = yamlNode["AutoWB"].as<bool>();
}

RosCamera::~RosCamera() {
    delete server;
    camera->Uninit();
}

void RosCamera::expCallback(const std_msgs::Int32ConstPtr& msg) {
    if (msg->data != internalConfig.ExpTime && msg->data > 500 && msg->data < 10000) {
        internalConfig.ExpTime = msg->data;
        camera->lock();
        camera->SetExposureTime(msg->data);
        if (internalConfig.AutoExp) {
            camera->SetExposureMode(false);
        }
        camera->unlock();
        server->updateConfig(internalConfig);
    }
}

void RosCamera::gainCallback(const std_msgs::Float64ConstPtr& msg) {
    if (msg->data != internalConfig.Gain && msg->data > 10 && msg->data < 300) {
        internalConfig.Gain = msg->data;
        camera->lock();
        camera->SetGain(msg->data);
        camera->unlock();
        server->updateConfig(internalConfig);
    }
}

void RosCamera::resolutionCallback(const std_msgs::StringConstPtr& msg) {
    if (msg->data != internalConfig.Resolution) {
        internalConfig.Resolution = msg->data;
        camera->lock();
        camera->SetResolution(msg->data);
        camera->unlock();
        server->updateConfig(internalConfig);
        updateCamInfo(camInfo, resolutionStructCreator(msg->data));
    }
}

void RosCamera::configCallback(md_camera::CameraConfig &_config, uint32_t _) {
    if (_config.ExpTime != internalConfig.ExpTime || !isInit) {
        camera->lock();
        camera->SetExposureTime(_config.ExpTime);
        camera->unlock();
    }

    if (_config.AutoExp != internalConfig.AutoExp || !isInit) {
        camera->lock();
        camera->SetExposureTime(_config.AutoExp);
        camera->unlock();
    }

    if (_config.Resolution != internalConfig.Resolution || !isInit) {
        camera->lock();
        camera->SetResolution(_config.Resolution);
        camera->unlock();
        updateCamInfo(camInfo, resolutionStructCreator(_config.Resolution));
    }

    if (_config.Gain != internalConfig.Gain || !isInit) {
        camera->lock();
        camera->SetGain(_config.Gain);
        camera->unlock();
    }

    if (_config.AutoWB == internalConfig.AutoWB || !isInit) {
        camera->lock();
        camera->SetWBMode(_config.AutoWB);
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

void RosCamera::cameraPubWorker() {
    ros::Rate loopRate(5);
    while(ros::ok()) {
        std_msgs::String msg;
        msg.data = internalConfig.CameraName;
        cameraNamePub.publish(msg);
        cameraInfoPub.publish(camInfo);
        loopRate.sleep();
    }
}

void RosCamera::getImageWorker() {
    auto lastImgTime = ros::Time::now();
    int errorCount = 0;
    camera->Play();

    while (ros::ok()) {
        if (errorCount > 5) {
            ROS_WARN("MD camera might drop, RECONNECT.");
            camera->Uninit();
            ros::shutdown();
        }
        cv::Mat raw_img;
        camera->lock();
        camera->GetFrame(raw_img);
        camera->unlock();
        if (raw_img.empty()) {
            ROS_WARN("NO IMG GOT FROM MD");
            lastImgTime = ros::Time::now();
            errorCount++;
            continue;
        }
        std_msgs::Header img_head;
        img_head.stamp = ros::Time::now();
        img_head.frame_id = "robot_md_camera";
        auto msg = cv_bridge::CvImage(img_head, "bgr8", raw_img).toImageMsg();
        imagePub.publish(msg);

        double diff = (ros::Time::now() - lastImgTime).toSec();
        if (diff > 2) {
            ROS_WARN("MD camera might drop, RECONNECT.");
            camera->Uninit();
            ros::shutdown();
        }
        lastImgTime = ros::Time::now();
    }
}

bool RosCamera::setCameraInfoCallback(sensor_msgs::SetCameraInfo::Request &req,
                                      sensor_msgs::SetCameraInfo::Response &res)
{
    camInfo = req.camera_info;
    CameraMatrix info = ci2cm(req.camera_info);
    camera->lock();
    int status = camera->SetCameraMatrix(info);
    camera->unlock();
    res.success = status == 0;
    res.status_message = "Already write to camera!";
    std::cout << "CAMERA CALIBRATION SUCCESS!" << std::endl;
    return true;
}

void RosCamera::save() {
    yamlNode["AutoExp"] = internalConfig.AutoExp;
    yamlNode["ExpTime"] = internalConfig.ExpTime;
    yamlNode["Resolution"] = internalConfig.Resolution;
    yamlNode["Gain"] = internalConfig.Gain;
    yamlNode["AutoWB"] = internalConfig.AutoWB;
    std::ofstream file(CONFIG_PATH);
    file << yamlNode;
}

void RosCamera::init() {
    ros::NodeHandle nh("~");
    std::string camera_name;
    nh.getParam("camera_name", camera_name);
    camera->Init(camera_name);
    camera->LoadParameters();
    camInfo = cm2ci(camera->GetCameraMatrix(), resolutionStructCreator(internalConfig.Resolution));
    server = new dynamic_reconfigure::Server<md_camera::CameraConfig>;
    expSub = nh.subscribe("exposure", 1, &RosCamera::expCallback, this);
    gainSub = nh.subscribe("gain", 1, &RosCamera::gainCallback, this);
    resolutionSub = nh.subscribe("resolution", 1, &RosCamera::resolutionCallback, this);
    cameraInfoService = nh.advertiseService("set_camera_info", &RosCamera::setCameraInfoCallback, this);
    cameraNamePub = nh.advertise<std_msgs::String>("camera_name", 1);
    cameraInfoPub = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
    imagePub = nh.advertise<sensor_msgs::Image>("raw_img", 2);
    internalConfig.CameraName = camera->GetCameraName();
    server->updateConfig(internalConfig);
    server->setCallback(boost::bind(&RosCamera::configCallback, this, _1, _2));

    if (yamlNode["HWTrigger"].as<bool>()) {
        camera->SetTriggerMode(MDCamera::hardware);
    } else {
        camera->SetTriggerMode(MDCamera::continuous);
    }

    isInit = true;
    cameraPubThread = std::thread(&RosCamera::cameraPubWorker, this);
    imagePubThread = std::thread(&RosCamera::getImageWorker, this);
}



