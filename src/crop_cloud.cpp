//
// Created by jeremy on 7/8/21.
//

#include "../include/crop_cloud.h"
void crop_cloud::crop(sensor_msgs::PointCloud2::Ptr input_cloud) {
    pcl::PCLPointCloud2::Ptr input_cloud_pcl (new pcl::PCLPointCloud2); //initialize PCL2 to be inputted into CropBox
    pcl_conversions::toPCL(*input_cloud, *input_cloud_pcl); //convert subscribed data (sensor_msgs/PCL2) into pcl::PCL2
    pcl::PCLPointCloud2 cropped_cloud_pcl; //initialize the PCL2 that will be outputted from CropBox

    pcl::CropBox<pcl::PCLPointCloud2> crop;
    crop.setMin(Eigen::Vector4f(-2.0, -0.18, 0.62, 1.0));
    crop.setMax(Eigen::Vector4f(2.0, 0.18, 0.92, 1.0));
    crop.setInputCloud(input_cloud_pcl);
    crop.filter(cropped_cloud_pcl);

    //pcl::PointCloud<pcl::PointXYZ>::Ptr crop_cloud::plane_seg_cloud;

    pcl::fromPCLPointCloud2(cropped_cloud_pcl, *crop_cloud::plane_seg_cloud); //convert cropped point cloud into pcl::PointXYZ to be planar segmented
}