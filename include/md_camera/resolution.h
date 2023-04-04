//
// Created by bismarck on 23-3-30.
//

#ifndef SRC_RESOLUTION_H
#define SRC_RESOLUTION_H

#include <opencv2/opencv.hpp>
#include <CameraDefine.h>
#include <string>

tSdkImageResolution resolutionStructCreator(const std::string& idx);
cv::Size resolutionSizeCreator(const std::string& idx);

#endif //SRC_RESOLUTION_H
