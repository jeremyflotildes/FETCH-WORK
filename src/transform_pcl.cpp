//
// Created by Jeremy on 7/29/21.
// cpp file to transform the point cloud to base_link

#include "../include/transform_pcl.h"

void transform_pcl::transform(sensor_msgs::PointCloud2::Ptr input_cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_pcl (new pcl::PointCloud<pcl::PointXYZ>); //converted from ros msg
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    //pcl_conversions::toPCL(*input_cloud, *input_cloud_pcl); //convert subscribed data (sensor_msgs/PCL2) into pcl::PCL2

    pcl::fromROSMsg(*input_cloud, *input_cloud_pcl);

    transformed_cloud->header.frame_id = "base_link";

    tf::StampedTransform transform;
    tf::TransformListener tf_listener;
    try {
        tf_listener.waitForTransform("/base_link", "/head_camera_rgb_optical_frame", ros::Time(0), ros::Duration(10.0));
        tf_listener.lookupTransform ("/base_link", "/head_camera_rgb_optical_frame", ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
    }
    pcl_ros::transformPointCloud(*input_cloud_pcl, *transformed_cloud, transform);

    toPCLPointCloud2(*transformed_cloud, *transformed_cloud_pcl); //convert transformed cloud into PCL2 for cropbox
}