#include <ros/ros.h>

// 引入自定义消息头文件
#include <byh_uav/uav_imu.h>
#include <byh_uav/uav_magnet.h>
#include <byh_uav/uav_frequence.h>
#include <byh_uav/uav_gps.h>

#include <livox_ros_driver2/CustomMsg.h>
#include <nlink_parser/LinktrackTagframe0.h>
#include "ultra_sound/UltraSound.h"
#include <sensor_msgs/Image.h>

// ---------------- 回调函数 ----------------
void imuCallback(const byh_uav::uav_imu::ConstPtr &msg, const std::string &name)
{
    ROS_INFO_STREAM("Received uav_imu from " << name
                                             << " timestamp=" << msg->header.stamp);
}

void magCallback(const byh_uav::uav_magnet::ConstPtr &msg, const std::string &name)
{
    ROS_INFO_STREAM("Received uav_magnet from " << name);
}

void gpsCallback(const byh_uav::uav_gps::ConstPtr &msg)
{
    ROS_INFO_STREAM("Received uav_gps, lat=" << msg->latitude
                                             << " lon=" << msg->longitude);
}

void lidarCallback(const livox_ros_driver2::CustomMsg::ConstPtr &msg)
{
    ROS_INFO_STREAM("Received Livox lidar with " << msg->point_num << " points");
}

void tagCallback(const nlink_parser::LinktrackTagframe0::ConstPtr &msg)
{
    ROS_INFO_STREAM("Received LinktrackTagframe0, id=" << (int)msg->id);
}

void ultrasoundCallback(const ultra_sound::UltraSound::ConstPtr &msg)
{
    ROS_INFO_STREAM("Received ultrasound at " << msg->header.stamp);
}

void imageCallback(const sensor_msgs::Image::ConstPtr &msg, const std::string &cam)
{
    ROS_INFO_STREAM("Received Image from " << cam
                                           << " size=" << msg->width << "x" << msg->height);
}

// ---------------- main ----------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "all_topics_subscriber");
    ros::NodeHandle nh;

    // UAV 自定义 IMU
    ros::Subscriber sub_adis = nh.subscribe<byh_uav::uav_imu>("/byh_uav/ADIS16470", 10,
                                                              boost::bind(&imuCallback, _1, "ADIS16470"));
    ros::Subscriber sub_bmi = nh.subscribe<byh_uav::uav_imu>("/byh_uav/BMI088", 10,
                                                             boost::bind(&imuCallback, _1, "BMI088"));

    // UAV 磁力计
    ros::Subscriber sub_ak8975 = nh.subscribe<byh_uav::uav_magnet>("/byh_uav/AK8975", 10,
                                                                   boost::bind(&magCallback, _1, "AK8975"));
    ros::Subscriber sub_rm3100 = nh.subscribe<byh_uav::uav_magnet>("/byh_uav/RM3100", 10,
                                                                   boost::bind(&magCallback, _1, "RM3100"));

    // UAV GPS
    ros::Subscriber sub_gps = nh.subscribe<byh_uav::uav_gps>("/byh_uav/ZEDF9P", 10, gpsCallback);

    // Livox IMU & Lidar
    ros::Subscriber sub_lidar = nh.subscribe<livox_ros_driver2::CustomMsg>("/livox/lidar", 10, lidarCallback);

    // UWB Tag
    ros::Subscriber sub_tag = nh.subscribe<nlink_parser::LinktrackTagframe0>("/nlink_linktrack_tagframe0", 10, tagCallback);

    // 相机
    ros::Subscriber sub_cam0 = nh.subscribe<sensor_msgs::Image>("/sensor/cam0", 10,
                                                                boost::bind(&imageCallback, _1, "cam0"));
    ros::Subscriber sub_cam1 = nh.subscribe<sensor_msgs::Image>("/sensor/cam1", 10,
                                                                boost::bind(&imageCallback, _1, "cam1"));

    ROS_INFO("All topic subscribers are ready.");
    ros::spin();
    return 0;
}
