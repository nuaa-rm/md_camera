//
// Created by bismarck on 23-4-5.
//

#ifndef SRC_RECORDER_H
#define SRC_RECORDER_H

#include <vector>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include "md_camera/TopicRecorder.h"
#include "md_camera/LockFrame.h"
#include "md_camera/cameraMatrix.h"


class Recorder {
private:
    cv::VideoWriter videoWriter;
    std::vector<TopicRecorder> topics;
    std::string path, now_path;
    CameraMatrix camMat{};
    size_t frame_count{0};
public:
    explicit Recorder(const std::string& _path);
    ~Recorder();

    std::string getRecordPath();

    void startRecord(const std::string& resolution, int recordFps, const sensor_msgs::CameraInfo& camInfo);
    void stopRecord();
    void pushRecordFrame(LockFrame* frame);
};


#endif //SRC_RECORDER_H
