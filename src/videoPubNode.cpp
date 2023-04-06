//
// Created by bismarck on 23-4-6.
//

#include <ros/ros.h>
#include "md_camera/Player.h"

Player player;

int main(int argc, char** argv) {
    ros::init(argc, argv, "md_camera_node");
    player.init();

    ros::spin();
}
