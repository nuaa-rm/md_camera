//
// Created by bismarck on 23-4-5.
//

#ifndef SRC_TOPICRECORDER_H
#define SRC_TOPICRECORDER_H

#include <atomic>
#include <string>
#include <fstream>

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <topic_tools/shape_shifter.h>

struct TopicProperties {
    std::string topic_name;
    std::string file_path;
    std::string datatype;
    std::string md5;
    std::string msg_def;
};

namespace YAML {
template<>
struct convert<TopicProperties> {
  static Node encode(const TopicProperties& info) {
    Node node;
    node["TopicName"] = info.topic_name;
    node["FilePath"] = info.file_path;
    node["Datatype"] = info.datatype;
    node["MD5"] = info.md5;
    node["MsgDef"] = info.msg_def;
    return node;
  }

  static bool decode(const Node& node, TopicProperties& info) {
    info.topic_name = node["TopicName"].as<std::string>();
    info.file_path = node["FilePath"].as<std::string>();
    info.datatype = node["Datatype"].as<std::string>();
    info.md5 = node["MD5"].as<std::string>();
    info.msg_def = node["MsgDef"].as<std::string>();
    return true;
  }
};
}

struct __attribute__((packed)) Head {
    size_t frame, size;
};

class TopicRecorder {
private:
    ros::Publisher pub;
    ros::Subscriber sub;

    std::function<void()> saveFunc;

    std::ifstream file_in;
    std::ofstream file_out;
    std::string file_path;

    char* buffer = nullptr;
    topic_tools::ShapeShifter msg;
    ros::serialization::OStream stream{(uint8_t*)buffer, 0};

    std::atomic<size_t> frame_count{0};
    Head last_head{};

    void callback(const topic_tools::ShapeShifter::ConstPtr& _msg);
public:
    enum class Mode {
        READ, WRITE
    };

    TopicRecorder() = default;
    TopicRecorder(const TopicProperties& info, TopicRecorder::Mode mode);
    TopicRecorder(TopicRecorder&) = delete;
    TopicRecorder(TopicRecorder&&) noexcept ;
    ~TopicRecorder();

    void init(const TopicProperties& info, TopicRecorder::Mode mode,
              std::function<void()> func=std::function<void()>());
    void reset();
    void close();
    size_t publish();
    TopicProperties getInfo();
    void setFrameCount(size_t _frame_count);
};


#endif //SRC_TOPICRECORDER_H
