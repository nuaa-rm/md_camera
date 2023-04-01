//
// Created by bismarck on 23-3-30.
//

#include <thread>
#include <md_camera/MDCamera.h>
#include <md_camera/config.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <yaml-cpp/yaml.h>

MDCamera camera;
Config config;
ros::Publisher image_pub;

void getImageWorker() {
    auto lastImgTime = ros::Time::now();
    int errorCount = 0;

    while (ros::ok()) {
        if (errorCount > 10) {
            ROS_WARN("MD camera might drop, RECONNECT.");
            camera.Uninit();
            ros::shutdown();
        }
        cv::Mat raw_img;
        camera.lock();
        camera.GetFrame(raw_img);
        camera.unlock();
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
        image_pub.publish(msg);

        double diff = (ros::Time::now() - lastImgTime).toSec();
        if (diff > 2) {
            ROS_WARN("MD camera might drop, RECONNECT.");
            camera.Uninit();
            ros::shutdown();
        }
        lastImgTime = ros::Time::now();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "md_camera_node");
    ros::NodeHandle nh("~");
    std::string camera_name;
    image_pub = nh.advertise<sensor_msgs::Image>("raw_img", 2);
    nh.getParam("camera_name", camera_name);
    camera.Init(camera_name);
    camera.LoadParameters();
    config.init(&camera);
    camera.Play();

    auto getImageThread = std::thread(getImageWorker);
    ros::spin();
    camera.Uninit();
}
