//
// Created by bismarck on 23-4-5.
//

#ifndef SRC_RECORDER_H
#define SRC_RECORDER_H

#include <vector>
#include <functional>
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
    std::string path, now_path, frame_id, camera_name;
    sensor_msgs::CameraInfo camInfo;
    size_t frame_count{0};
    int frame_rate;
    bool recording{false};

    void saveYaml();
public:
    explicit Recorder(const std::string& _path);
    ~Recorder();

    std::string getRecordPath();
    std::function<void()> getSaveFunc();

    void startRecord(const std::string& resolution, int recordFps, const sensor_msgs::CameraInfo& _camInfo,
                     const std::string& _frame_id, const std::string& _camera_name);
    void stopRecord();
    void pushRecordFrame(LockFrame* frame);
};


#endif //SRC_RECORDER_H
