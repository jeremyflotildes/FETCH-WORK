

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/extract_clusters.h>


#ifndef CROP_PC_PLANAR_SEGMENTATION_H
#define CROP_PC_PLANAR_SEGMENTATION_H
class planar_segmentation {
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_extracted;
    void extractOffPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr plane_seg_cloud);

    planar_segmentation() : cloud_extracted(new pcl::PointCloud<pcl::PointXYZ>) { //initialize point cloud in the constructor -- solve errors (undefined references) with linkers across c++ files
    }
};
#endif //CROP_PC_PLANAR_SEGMENTATION_H
