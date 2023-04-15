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
#define SUFFIX ".mp4"
#endif

#ifdef MJPG
#define FOUR_CC cv::VideoWriter::fourcc('M','J','P','G')
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

template <typename U>
struct AccelerationEnable {
	template <typename T, typename T2 = decltype(T::VIDEOWRITER_PROP_HW_ACCELERATION)>
	static constexpr bool check(T2) { return true; };

    template<typename T>
	static constexpr bool check(...) { return false; };

	static constexpr bool ret = check<U>(0);
};

template<typename T = cv::VideoWriterProperties, typename T2 = cv::VideoAccelerationType>
typename std::enable_if<AccelerationEnable<T>::ret, void>::type
startVideoRecorder(cv::VideoWriter& videoWriter, const std::string& now_path, int recordFps, cv::Size size) {
    std::vector<int> params{T::VIDEOWRITER_PROP_HW_ACCELERATION, T2::VIDEO_ACCELERATION_ANY};
    videoWriter.open(now_path + "video" + SUFFIX, FOUR_CC, recordFps, size, params);
}

template<typename T = cv::VideoWriterProperties>
typename std::enable_if<!AccelerationEnable<T>::ret, void>::type
startVideoRecorder(cv::VideoWriter& videoWriter, const std::string& now_path, int recordFps, cv::Size size) {
    std::cerr << "Opencv Version not Support Set VideoWriter ACCELERATION!" << std::endl;
    videoWriter.open(now_path + "video" + SUFFIX, FOUR_CC, recordFps, size);
}

void Recorder::startRecord(const std::string &resolution, int recordFps, const sensor_msgs::CameraInfo& _camInfo,
                           const std::string& _frame_id, const std::string& _camera_name)
{
    ros::NodeHandle nh("~");
    now_path = getRecordPath();
    camInfo = _camInfo;
    frame_id = _frame_id;
    frame_rate = recordFps;
    camera_name = _camera_name;
    cv::Size size = resolutionSizeCreator(resolution);
    startVideoRecorder(videoWriter, now_path, recordFps, size);
    auto saveFunc = [this](){
        this->saveYaml();
    };
    if (nh.hasParam("record_topics")) {
        XmlRpc::XmlRpcValue v;
        nh.getParam("record_topics", v);
        topics.resize(v.size());
        for (int i = 0; i < v.size(); i++) {
            topics[i].init(TopicProperties{v[i], now_path + "topic_" + std::to_string(i) + ".mbg"},
                           TopicRecorder::Mode::WRITE, saveFunc);
        }
    }
    saveYaml();
    std::cout << "VIDEO RECORD START !!" << std::endl;
    std::cout << "Video save to " << now_path << std::endl;
    recording = true;
}

void Recorder::stopRecord() {
    if (!recording) {
        std::cout << "NOT RECORDING" << std::endl;
        return;
    }
    videoWriter.release();
    topics.clear();
    std::cout << "VIDEO RECORD STOP !!" << std::endl;
    recording = false;
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

void Recorder::saveYaml() {
    YAML::Node node;
    for (int i = 0; i < topics.size(); i++) {
        auto info = topics[i].getInfo();
        if (!info.md5.empty()) {
            info.file_path = "topic_" + std::to_string(i) + ".mbg";
            node["topics"].push_back(info);
        }
    }
    node["video"] = std::string("video") + SUFFIX;
    node["cameraMatrix"] = camInfo;
    node["frameId"] = frame_id;
    node["cameraName"] = camera_name;
    node["frameRate"] = frame_rate;
    std::ofstream file(now_path + "info.yaml");
    file << node;
    file.close();
    std::cout << "RECORD YAML SAVED !!" << std::endl;
}
