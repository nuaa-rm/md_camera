# MindVision相机ROS驱动
采用ROS标准接口的迈徳威视相机驱动包

## 特色功能
* 可以直接使用ROS CameraCalibrate进行标定，并将标定结果储存到相机中， 
实现在自己电脑上标定后直接部署，不需要转移参数，节点通过topic和service的方式广播相机参数
* 支持节点多开，通过相机名识别目标相机，相机名也将通过topic和service广播，供其他程序使用
* 支持在线调参，可以通过dynamic_reconfigure和topic的方式对部分常用参数动态调参，
支持直接保存当前参数到配置文件中
* 支持预设分辨率参数（包含宽高和ROI），可以通过topic和dynamic_reconfigure直接设置为指定配置
* 支持设置触发方式（暂时只支持连续模式和硬件触发）

## 相关依赖
* [yaml-cpp](https://github.com/jbeder/yaml-cpp)
* [MindVision SDK](https://mindvision.com.cn/rjxz/list_12.aspx?lcid=138)

## 使用方法
* 本仓库可以直接被子模块链接到工程中，示例：
    ```shell
    git submodule add git@github.com:nuaa-rm/md_camera.git 
    ```
* 在工作空间src目录下直接clone本仓库

## API
* Topic Publish
  * ~raw_img (sensor_msgs/Image)
  * ~camera_info (sensor_msgs/CameraInfo)
  * ~camera_name (std_msgs/String)
* Topic Subscribe
  * ~exposure (std_msgs/Int32)  unit: us  range: 500\~5000
  * ~gain (std_msgs/Float64)  range: 10\~300
  * ~resolution (std_msgs/String)  must in `resolution.yaml`
  * ~record (std_msgs/Bool)  start or stop record
* Service
  * ~set_camera_info (sensor_msgs/setCameraInfo)
  * ~get_camera_info (ms_camera/getCameraInfo)
* Param
  * ~camera_name (string)  set empty to any
  * ~frame_id (string)  ros frame id
  * ~config_file (string)  default: config.yaml, The file must be in params folder

## RoadMap
* 计划支持帧率控制功能
