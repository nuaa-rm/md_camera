//
// Created by bismarck on 23-3-30.
//

#include <ros/ros.h>
#include "md_camera/resolution.h"

tSdkImageResolution resolutionStructCreator(const std::string& idx) {
    tSdkImageResolution res;
    if (idx == "MaxSize") {
        res.iIndex = 0;
        res.iWidth = 1280;
        res.iHeight = 1024;
        res.iHOffsetFOV = 0;
        res.iVOffsetFOV = 0;
    } else if (idx == "640_480") {
        res.iIndex = 1;
        res.iWidth = 640;
        res.iHeight = 480;
        res.iHOffsetFOV = 320;
        res.iVOffsetFOV = 272;
    } else {
        int width, height, offset_w, offset_h;
        ros::param::get("/Resolution/" + idx + "/width", width);
        ros::param::get("/Resolution/" + idx + "/height", height);
        ros::param::get("/Resolution/" + idx + "/offset_W", offset_w);
        ros::param::get("/Resolution/" + idx + "/offset_H", offset_h);
        // 设置成0xff表示自定义分辨率，设置成0到N表示选择预设分辨率
        // Set to 0xff for custom resolution, set to 0 to N for select preset resolution
        res.iIndex = 0xff;

        // iWidthFOV表示相机的视场宽度，iWidth表示相机实际输出宽度
        // 大部分情况下iWidthFOV=iWidth。有些特殊的分辨率模式如BIN2X2：iWidthFOV=2*iWidth，表示视场是实际输出宽度的2倍
        // iWidthFOV represents the camera's field of view width, iWidth represents the camera's actual output width
        // In most cases iWidthFOV=iWidth. Some special resolution modes such as BIN2X2:iWidthFOV=2*iWidth indicate that the field of view is twice the actual output width

        res.iWidth = width;
        res.iWidthFOV = width;

        // 高度，参考上面宽度的说明
        // height, refer to the description of the width above
        res.iHeight = height;
        res.iHeightFOV =height;

        // 视场偏移
        // Field of view offset
        res.iHOffsetFOV = offset_w;
        res.iVOffsetFOV = offset_h;

        // ISP软件缩放宽高，都为0则表示不缩放
        // ISP software zoom width and height, all 0 means not zoom
        res.iWidthZoomSw = 0;
        res.iHeightZoomSw = 0;

        // BIN SKIP 模式设置（需要相机硬件支持）
        // BIN SKIP mode setting (requires camera hardware support)
        res.uBinAverageMode = 0;
        res.uBinSumMode = 0;
        res.uResampleMask = 0;
        res.uSkipMode =  0;
    }
    return res;
}

cv::Size resolutionSizeCreator(const std::string& idx) {
    cv::Size res;
    if (idx == "MaxSize") {
        res.width = 1280;
        res.height = 1024;
    } else if (idx == "640_480") {
        res.width = 640;
        res.height = 480;
    } else {
        int width, height;
        ros::param::get("/Resolution/" + idx + "/width", width);
        ros::param::get("/Resolution/" + idx + "/height", height);
        res.width = width;
        res.height = height;
    }
    return res;
}
