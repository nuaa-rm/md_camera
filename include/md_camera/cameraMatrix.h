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

sensor_msgs::CameraInfo cm2ci(CameraMatrix in, tSdkImageResolution resolution);
void updateCamInfo(sensor_msgs::CameraInfo& res, tSdkImageResolution resolution);
CameraMatrix ci2cm(sensor_msgs::CameraInfo in);
void reComputeCamInfo(sensor_msgs::CameraInfo& in);

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

template<>
struct convert<sensor_msgs::CameraInfo> {
  static Node encode(const sensor_msgs::CameraInfo& info) {
    Node node;
    node["height"] = info.height;
    node["width"] = info.width;
    node["roi"]["height"] = info.roi.height;
    node["roi"]["width"] = info.roi.width;
    node["roi"]["offset_h"] = info.roi.y_offset;
    node["roi"]["offset_w"] = info.roi.x_offset;
    node["fx"] = (float)info.K[0];
    node["fy"] = (float)info.K[4];
    node["cx"] = (float)info.K[2];
    node["cy"] = (float)info.K[5];
    node["distcc"].push_back((float)info.D[0]);
    node["distcc"].push_back((float)info.D[1]);
    node["distcc"].push_back((float)info.D[2]);
    node["distcc"].push_back((float)info.D[3]);
    return node;
  }

  static bool decode(const Node& node, sensor_msgs::CameraInfo& info) {
    info.height = node["height"].as<int>();
    info.width = node["width"].as<int>();
    info.roi.height = node["roi"]["height"].as<int>();
    info.roi.width = node["roi"]["width"].as<int>();
    info.roi.y_offset = node["roi"]["offset_h"].as<int>();
    info.roi.x_offset = node["roi"]["offset_w"].as<int>();
    info.K = {
            node["fx"].as<float>(),     0,                          node["cx"].as<float>(),
            0,                          node["fy"].as<float>(),     node["cy"].as<float>(),
            0,                          0,                          1
    };
    info.R = {
            1, 0, 0,
            0, 1, 0,
            0, 0, 1
    };
    info.distortion_model = "pin_hole";
    info.D = {node["distcc"][0].as<float>(), node["distcc"][1].as<float>(),
            node["distcc"][2].as<float>(), node["distcc"][3].as<float>(), 0};
    reComputeCamInfo(info);
    return true;
  }
};
}

#endif //SRC_CAMERAMATRIX_H
