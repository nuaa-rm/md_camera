//
// Created by bismarck on 23-3-30.
//

#include <md_camera/MDCamera.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

MDCamera camera;
ros::Publisher image_pub;

int main(int argc, char **argv) {
    ros::init(argc,argv,"mv_camera_node");
    ros::NodeHandle nh("~");
    image_pub = nh.advertise<sensor_msgs::Image>("raw_img", 2);
    camera.Init();
    camera.LoadParameters();
    camera.SetTriggerMode(MDCamera::hardware);
    camera.SetExposureTime(false, 5000);
    camera.SetResolution("640_480");
    camera.Play();
    auto lastImgTime = ros::Time::now();

    while (ros::ok())
    {
        cv::Mat raw_img;
        camera.GetFrame(raw_img);
        if(raw_img.empty())
        {
            ROS_WARN("NO IMG GOT FROM MV");
        }
        std_msgs::Header img_head;
        img_head.stamp = ros::Time::now();
        img_head.frame_id = "robot_md_camera";
        auto msg = cv_bridge::CvImage(img_head, "bgr8", raw_img).toImageMsg();
        image_pub.publish(msg);

        float diff=(ros::Time::now() - lastImgTime).toNSec()/1e9;
        if(diff>0.5)
        {
            ROS_WARN("MVcamera might drop, RECONNECT.");
            ros::shutdown();
        }
        lastImgTime = ros::Time::now();
        ros::spinOnce();
    }
}
