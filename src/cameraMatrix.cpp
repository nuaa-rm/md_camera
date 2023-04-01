//
// Created by bismarck on 23-4-1.
//

#include <opencv2/opencv.hpp>
#include <md_camera/cameraMatrix.h>


sensor_msgs::CameraInfo cm2ci(CameraMatrix in, tSdkImageResolution resolution) {
    sensor_msgs::CameraInfo res;
    res.width = 1280;
    res.height = 1024;
    res.distortion_model = "pin_hole";
    res.D = {in.d1, in.d2, in.d3, in.d4, 0};
    res.K = {
            in.fx,  0,      in.cx,
            0,      in.fy,  in.cy,
            0,      0,      1
    };
    res.R = {
            1, 0, 0,
            0, 1, 0,
            0, 0, 1
    };
    res.roi.height = resolution.iHeight;
    res.roi.width = resolution.iWidth;
    res.roi.x_offset = resolution.iHOffsetFOV;
    res.roi.y_offset = resolution.iVOffsetFOV;
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << in.fx, 0, in.cx, 0, in.fy, in.cy, 0, 0, 1);
    cv::Mat dist = (cv::Mat_<double>(1, 4) << in.d1, in.d2, in.d3, in.d4);
    cv::Mat p = cv::getOptimalNewCameraMatrix(cameraMatrix, dist, cv::Size((int)res.width, (int)res.height), 0);
    res.P = {
        p.at<double>(0,0), p.at<double>(0,1), p.at<double>(0,2), p.at<double>(0,3),
        p.at<double>(1,0), p.at<double>(1,1), p.at<double>(1,2), p.at<double>(1,3),
        p.at<double>(2,0), p.at<double>(2,1), p.at<double>(2,2), p.at<double>(2,3)
    };
    return res;
}

CameraMatrix ci2cm(sensor_msgs::CameraInfo in) {
    CameraMatrix res{};
    res.fx = (float)in.K[0];
    res.fy = (float)in.K[4];
    res.cx = (float)in.K[2];
    res.cy = (float)in.K[5];
    res.d1 = (float)in.D[0];
    res.d2 = (float)in.D[1];
    res.d3 = (float)in.D[2];
    res.d4 = (float)in.D[3];
    return res;
}
