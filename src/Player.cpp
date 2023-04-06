//
// Created by bismarck on 23-4-6.
//

#include <cv_bridge/cv_bridge.h>
#include "md_camera/Player.h"

Player::~Player() {
    capture.release();
    if (cameraInfoPubThread.joinable()) {
        cameraInfoPubThread.join();
    }
    if (imagePubThread.joinable()) {
        imagePubThread.join();
    }
}

void Player::init() {
    ros::NodeHandle nh("~");
    std::string record_dir;
    nh.param("record_dir", record_dir, std::string());
    path = CONFIG_PATH + record_dir + '/';

    YAML::Node node = YAML::LoadFile(path + "info.yaml");
    video_path = path + node["video"].as<std::string>();
    camInfo = node["cameraMatrix"].as<sensor_msgs::CameraInfo>();
    cameraName = node["cameraName"].as<std::string>();
    maxFrameCount = node["frameCount"].as<size_t>();
    frameId = node["frameId"].as<std::string>();
    YAML::Node topicNode = node["topics"];

    cameraNamePub = nh.advertise<std_msgs::String>("camera_name", 1);
    cameraInfoPub = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
    imagePub = nh.advertise<sensor_msgs::Image>("raw_img", 2);
    getCameraInfoService = nh.advertiseService("get_camera_info", &Player::getCameraInfoCallback, this);

    std::vector<int> params{cv::VIDEOWRITER_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_ANY};
    capture.open(video_path, cv::CAP_ANY, params);

    cameraInfoPubThread = std::thread(&Player::cameraInfoPubWorker, this);
    imagePubThread = std::thread(&Player::imagePubWorker, this);
}

void Player::reset() {
    capture.release();
    frameCount = 0;
    for (auto& topic: topics) {
        topic.reset();
    }
    std::vector<int> params{cv::VIDEOWRITER_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_ANY};
    capture.open(video_path, cv::CAP_ANY, params);
}

void Player::cameraInfoPubWorker() {
    ros::Rate loopRate(5);
    std_msgs::String msg;
    msg.data = cameraName;
    while(ros::ok()) {
        cameraNamePub.publish(msg);
        cameraInfoPub.publish(camInfo);
        loopRate.sleep();
    }
}

void Player::imagePubWorker() {
    std::vector<size_t> next_pub;
    next_pub.resize(topics.size(), 0);
    cv::Mat image;
    std_msgs::Header img_head;
    img_head.frame_id = frameId;
    while (ros::ok()) {
        for (int i = 0; i < next_pub.size(); i++) {
            if (frameCount >= next_pub[i]) {
                next_pub[i] = topics[i].publish();
            }
        }
        capture.read(image);
        img_head.stamp = ros::Time::now();
        auto msg = cv_bridge::CvImage(img_head, "bgr8", image).toImageMsg();
        imagePub.publish(msg);
        frameCount++;
        if (frameCount >= maxFrameCount) {
            std::cout << "Video Play to End. Replay !!!" << std::endl;
            reset();
        }
    }
}

bool Player::getCameraInfoCallback(md_camera::GetCameraInfo::Request &_, md_camera::GetCameraInfo::Response &res) {
    res.camera_name = cameraName;
    res.camera_info = camInfo;
    return true;
}


