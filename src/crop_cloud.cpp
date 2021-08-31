//
// Created by jeremy on 7/8/21.
// crops everything but the relevant plane with objects

#include "../include/crop_cloud.h"
void crop_cloud::crop(pcl::PCLPointCloud2::Ptr transformed_cloud) {
   /* pcl::PCLPointCloud2::Ptr input_cloud_pcl (new pcl::PCLPointCloud2); //initialize PCL2 to be inputted into CropBox
    pcl_conversions::toPCL(*input_cloud, *input_cloud_pcl); //convert subscribed data (sensor_msgs/PCL2) into pcl::PCL2 */

    pcl::PCLPointCloud2 cropped_cloud_pcl; //initialize the PCL2 that will be outputted from CropBox

    pcl::CropBox<pcl::PCLPointCloud2> crop;
/*    crop.setMin(Eigen::Vector4f(0.4, -0.58, 0.52, 1.0));
    crop.setMax(Eigen::Vector4f(0.8, 0.58, 0.82, 1.0));*/
    crop.setMin(Eigen::Vector4f(-1.0, -1.0, 0.80, 1.0));
    crop.setMax(Eigen::Vector4f(1.0, 0.5, 1.0, 1.0));
    crop.setInputCloud(transformed_cloud);
    crop.filter(cropped_cloud_pcl);
    
    //pcl::PointCloud<pcl::PointXYZ>::Ptr plane_seg_cloud;
    pcl::fromPCLPointCloud2(cropped_cloud_pcl, *plane_seg_cloud); //convert cropped point cloud into pcl::PointXYZ to be planar segmented
}