//
// Created by jeremy on 7/8/21.
//header file that declares the class used for cropping
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <ros/ros.h>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>

#include <pcl/filters/crop_box.h>


#ifndef CROP_PC_CROP_CLOUD_H
#define CROP_PC_CROP_CLOUD_H
class crop_cloud {
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_seg_cloud; //initialize new PointXYZ for plane segmentation, result for the crop will be stored here
    void crop(sensor_msgs::PointCloud2::Ptr input_cloud);
    crop_cloud() : plane_seg_cloud(new pcl::PointCloud<pcl::PointXYZ>){ //initialize point cloud in the constructor -- solve errors (undefined references) with linkers across c++ files
    }
};
#endif //CROP_PC_CROP_CLOUD_H
