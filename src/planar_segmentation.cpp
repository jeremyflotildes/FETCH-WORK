//
// Created by jeremy on 7/8/21.
// file that performs plane segmentation, then extracts objects off the plane

#include "../include/planar_segmentation.h"
void planar_segmentation::extractOffPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr plane_seg_cloud) {

    //---PLANE SEGMENTATION---
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (plane_seg_cloud);
    seg.segment (*inliers, *coefficients);

    //---EXTRACT POINTS ON/OFF THE PLANE---
    //Create filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    // Extract the inliers
    extract.setInputCloud (plane_seg_cloud);
    extract.setIndices (inliers);
    extract.setNegative (true); //false -- all inliers (the plane), true -- non-inliers (off the plane)
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_extracted;
    extract.filter(*cloud_extracted);
}