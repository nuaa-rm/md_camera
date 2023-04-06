//
// Created by bismarck on 23-3-30.
//

#include <md_camera/rosCamera.h>
#include <ros/ros.h>
#include <unistd.h>

RosCamera camera;

int main(int argc, char **argv) {
    ros::init(argc, argv, "md_camera_node");
    camera.init();

    ros::spinOnce();

    sleep(10);

    ros::shutdown();
}
