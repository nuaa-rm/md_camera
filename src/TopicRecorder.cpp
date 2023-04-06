//
// Created by bismarck on 23-4-5.
//

#include "md_camera/TopicRecorder.h"


TopicRecorder::TopicRecorder(const TopicProperties& info, TopicRecorder::Mode mode) {
    init(info, mode);
}

TopicRecorder::~TopicRecorder() {
    close();
}

TopicRecorder::TopicRecorder(TopicRecorder && other) noexcept {
    pub = other.pub;
    sub = other.sub;
    file_in.swap(other.file_in);
    file_out.swap(other.file_out);
    buffer = other.buffer;
    msg = other.msg;
    stream = other.stream;
    frame_count = other.frame_count.load();

    other.pub = ros::Publisher();
    other.sub = ros::Subscriber();
    other.buffer = nullptr;
    other.stream = ros::serialization::OStream(nullptr, 0);
}

void TopicRecorder::init(const TopicProperties& info, TopicRecorder::Mode mode) {
    ros::NodeHandle nh;
    file_path = info.file_path;
    if (mode == Mode::READ) {
        msg.morph(info.md5, info.datatype, info.msg_def, "");
        pub = msg.advertise(nh, info.topic_name, 1);
        file_in.open(info.file_path, std::ios::in | std::ios::binary);
    } else if (mode == Mode::WRITE) {
        sub = nh.subscribe(info.topic_name, 1, &TopicRecorder::callback, this);
        file_out.open(info.file_path, std::ios::out | std::ios::binary);
    }
}

void TopicRecorder::close() {
    sub.shutdown();
    delete[] buffer;
    buffer = nullptr;
    stream = ros::serialization::OStream(nullptr, 0);
    if (file_in.is_open()) {
        file_in.close();
    }
    if (file_out.is_open()) {
        file_out.close();
    }
}

void TopicRecorder::callback(const topic_tools::ShapeShifter::ConstPtr &_msg) {
    if (buffer == nullptr) {
        buffer = new char[_msg->size()+32];
        msg = *_msg;
        stream = ros::serialization::OStream((uint8_t*)buffer, msg.size()+32);
    }
    _msg->write(stream);
    Head head{frame_count, _msg->size()};
    file_out.write((char*)&head, sizeof(Head));
    file_out.write(buffer, (long)head.size);
}

size_t TopicRecorder::publish() {
    if (buffer == nullptr) {
        file_in.read((char*)&last_head, sizeof(Head));
        buffer = new char[last_head.size+32];
    } else {
        stream = ros::serialization::OStream((uint8_t*)buffer, last_head.size);
        msg.read(stream);
        pub.publish(msg);
        file_in.read((char*)&last_head, sizeof(Head));
    }
    return last_head.frame;
}

TopicProperties TopicRecorder::getInfo() {
    return TopicProperties{
        sub.getTopic(),
        "",
        msg.getDataType(),
        msg.getMD5Sum(),
        msg.getMessageDefinition()
    };
}

void TopicRecorder::setFrameCount(size_t _frame_count) {
    frame_count = _frame_count;
}

void TopicRecorder::reset() {
    sub.shutdown();
    delete[] buffer;
    buffer = nullptr;
    stream = ros::serialization::OStream(nullptr, 0);
    if (file_in.is_open()) {
        file_in.close();
        file_in.open(file_path, std::ios::in | std::ios::binary);
    }
    if (file_out.is_open()) {
        file_out.close();
        file_out.open(file_path, std::ios::out | std::ios::binary);
    }
    frame_count = 0;
}
