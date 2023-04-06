//
// Created by bismarck on 23-4-1.
//

#ifndef SRC_CAMERAMATRIX_H
#define SRC_CAMERAMATRIX_H

#include <CameraDefine.h>
#include <yaml-cpp/yaml.h>
#include <sensor_msgs/CameraInfo.h>

struct __attribute__((packed)) CameraMatrix {
    float fx, fy, cx, cy, d1, d2, d3, d4;
};

namespace YAML {
template<>
struct convert<CameraMatrix> {
  static Node encode(const CameraMatrix& info) {
    Node node;
    node["fx"] = info.fx;
    node["fy"] = info.fy;
    node["cx"] = info.cx;
    node["cy"] = info.cy;
    node["distcc"].push_back(info.d1);
    node["distcc"].push_back(info.d2);
    node["distcc"].push_back(info.d3);
    node["distcc"].push_back(info.d4);
    return node;
  }

  static bool decode(const Node& node, CameraMatrix& info) {
    if(!node.IsSequence() || node.size() != 5) {
      return false;
    }
    info.fx = node["fx"].as<float>();
    info.fy = node["fy"].as<float>();
    info.cx = node["cx"].as<float>();
    info.cy = node["cy"].as<float>();
    info.d1 = node["distcc"][0].as<float>();
    info.d2 = node["distcc"][1].as<float>();
    info.d3 = node["distcc"][2].as<float>();
    info.d4 = node["distcc"][3].as<float>();
    return true;
  }
};
}

sensor_msgs::CameraInfo cm2ci(CameraMatrix in, tSdkImageResolution resolution);
void updateCamInfo(sensor_msgs::CameraInfo& res, tSdkImageResolution resolution);
CameraMatrix ci2cm(sensor_msgs::CameraInfo in);

#endif //SRC_CAMERAMATRIX_H
