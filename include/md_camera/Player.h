//
// Created by bismarck on 23-4-6.
//

#ifndef SRC_PLAYER_H
#define SRC_PLAYER_H

#include <thread>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <md_camera/GetCameraInfo.h>
#include "md_camera/TopicRecorder.h"
#include "md_camera/cameraMatrix.h"

class Player {
private:
    cv::VideoCapture capture;
    std::vector<TopicRecorder> topics;
    sensor_msgs::CameraInfo camInfo;
    size_t frameCount{0}, maxFrameCount{0};
    std::string video_path, frameId;
    std::string path, cameraName;

    ros::Publisher cameraInfoPub;
    ros::Publisher cameraNamePub;
    ros::Publisher imagePub;
    ros::ServiceServer getCameraInfoService;

    std::thread cameraInfoPubThread;
    std::thread imagePubThread;

    void reset();
    void cameraInfoPubWorker();
    void imagePubWorker();
    bool getCameraInfoCallback(md_camera::GetCameraInfo::Request &_, md_camera::GetCameraInfo::Response &res);
public:
    Player() = default;
    ~Player();

    void init();
};


#endif //SRC_PLAYER_H
