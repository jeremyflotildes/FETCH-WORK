//
// Created by Jeremy on 7/29/21.
// transform the entire point cloud from head camera -> base_link

#include <ros/ros.h>

#include <tf_conversions/tf_eigen.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include "pcl_ros/transforms.h"

#include <tf/transform_broadcaster.h>
#include "tf/transform_listener.h"

#include <sensor_msgs/PointCloud2.h>

#ifndef CROP_PC_TRANSFORM_PCL_H
#define CROP_PC_TRANSFORM_PCL_H
class transform_pcl {
public:
    pcl::PCLPointCloud2::Ptr transformed_cloud_pcl;  //initialize new PCL2 for transformation, result of the transformation will be stored here and passed to other functions
    void transform(sensor_msgs::PointCloud2::Ptr input_cloud);
    transform_pcl() : transformed_cloud_pcl (new pcl::PCLPointCloud2) { //initialize point cloud in the constructor -- solve errors (undefined references) with linkers across c++ files
    }
};
#endif //CROP_PC_TRANSFORM_PCL_H
