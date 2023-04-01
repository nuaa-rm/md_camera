//
// Created by bismarck on 23-4-1.
//

#ifndef SRC_CAMERAMATRIX_H
#define SRC_CAMERAMATRIX_H

#include <CameraDefine.h>
#include <sensor_msgs/CameraInfo.h>

struct __attribute__((packed)) CameraMatrix {
    float fx, fy, cx, cy, d1, d2, d3, d4;
};

sensor_msgs::CameraInfo cm2ci(CameraMatrix in, tSdkImageResolution resolution);
void updateCamInfo(sensor_msgs::CameraInfo& res, tSdkImageResolution resolution);
CameraMatrix ci2cm(sensor_msgs::CameraInfo in);

#endif //SRC_CAMERAMATRIX_H
