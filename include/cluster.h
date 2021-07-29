//
// Created by jeremy on 7/10/21.
// header file that declares the class used for cluster extraction

#include <iostream>

#include <ros/ros.h>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <tf/transform_broadcaster.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <pcl/common/common.h>

#include <Eigen/Eigenvalues>


#ifndef CROP_PC_CLUSTER_H
#define CROP_PC_CLUSTER_H
class cluster_extraction {
public:
    Eigen::Vector4d centroid;
/*    static Eigen::Vector4f min;
    static Eigen::Vector4f max;
    static float length;
    static float width;
    static float height;*/
    void cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_extracted);
    cluster_extraction() {
    }
};
#endif //CROP_PC_CLUSTER_H
