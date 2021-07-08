//
// Created by jeremy on 7/8/21.
//
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
    static pcl::PointCloud<pcl::PointXYZ>::Ptr plane_seg_cloud; //initialize new PointXYZ for plane segmentation
    void crop(sensor_msgs::PointCloud2::Ptr input_cloud);
};
#endif //CROP_PC_CROP_CLOUD_H
