//
// Created by bismarck on 23-4-5.
//

#include <ctime>
#include <sys/stat.h>
#include <unistd.h>
#include "md_camera/resolution.h"
#include "md_camera/Recorder.h"

#define X264

#ifdef X264
#define FOUR_CC cv::VideoWriter::fourcc('X','2','6','4')
#define SUFFIX ".mkv"
#endif

#ifdef MJPG
#define FOUR_CC_MJPG cv::VideoWriter::fourcc('M','J','P','G')
#define SUFFIX ".avi"
#endif


std::string Recorder::getRecordPath() {
    tm stime{};
    time_t now = time(nullptr);
    localtime_r(&now, &stime);

    char tmp[32] = {0};
    strftime(tmp, sizeof(tmp), "%Y-%m-%d_%H-%M-%S/", &stime);
    std::string _path = path + tmp;
    if (access(_path.c_str(), F_OK)) {
        mkdir(_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }
    return path + tmp;
}

void Recorder::startRecord(const std::string &resolution, int recordFps, const sensor_msgs::CameraInfo& camInfo) {
    ros::NodeHandle nh("~");
    XmlRpc::XmlRpcValue v;
    now_path = getRecordPath();
    camMat = ci2cm(camInfo);
    cv::Size size = resolutionSizeCreator(resolution);
    std::vector<int> params{cv::VIDEOWRITER_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_ANY};
    videoWriter.open(now_path + "video" + SUFFIX, FOUR_CC, recordFps, size, params);
    if (nh.hasParam("record_topics")) {
        nh.getParam("record_topics", v);
        topics.resize(v.size());
        for (int i = 0; i < v.size(); i++) {
            topics[i].init(TopicProperties{v[i], now_path + "topic_" + std::to_string(i) + ".mbg"},
                           TopicRecorder::Mode::WRITE);
        }
    }
    std::cout << "VIDEO RECORD START !!" << std::endl;
    std::cout << "Video save to " << now_path << std::endl;
}

void Recorder::stopRecord() {
    videoWriter.release();
    YAML::Node node;
    for (int i = 0; i < topics.size(); i++) {
        auto info = topics[i].getInfo();
        topics[i].close();
        if (!info.md5.empty()) {
            info.file_path = now_path + "topic_" + std::to_string(i) + ".mbg";
            node["topics"].push_back(info);
        }
    }
    node["video"] = std::string("video") + SUFFIX;
    node["frameCount"] = frame_count;
    node["cameraMatrix"] = camMat;
    topics.clear();
    std::ofstream file(now_path + "info.yaml");
    file << node;
    file.close();
    std::cout << "VIDEO RECORD STOP !!" << std::endl;
}

void Recorder::pushRecordFrame(LockFrame *frame) {
    cv::Mat raw_img = cv::Mat(
        cv::Size(frame->headPtr()->iWidth, frame->headPtr()->iHeight),
        frame->headPtr()->uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
        frame->data()
    );
    videoWriter.write(raw_img);
    frame_count++;
    for (auto& topic: topics) {
        topic.setFrameCount(frame_count);
    }
}

Recorder::Recorder(const std::string& _path) {
    path = _path;
}

Recorder::~Recorder() {
    stopRecord();
}
