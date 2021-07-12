//
// Created by jeremy on 7/10/21.
// function that extracts clusters, calculates their centroids, transforms centroids with respect to base_link, and broadcasts them

#include "cluster.h"
void cluster_extraction::cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_extracted) { //fixed: "undefined reference to cluster_extraction::cluster" -> function requires parent class + scope operator
    // ---CREATING THE KDTREE OBJECT FOR THE SEARCH METHOD OF THE EXTRACTION---
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_extracted);

    std::vector<pcl::PointIndices> cluster_indices; //vector containing one instance of PointIndices for each detected cluster
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.015); // 1.5 cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_extracted);
    ec.extract (cluster_indices);

    int j = 0;
    Eigen::Vector4d centroid;
    tf::TransformBroadcaster br;
    tf::TransformListener listener;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) //iterate thru cluster_indices (each element of cluster_indices contains all the points of a cluster)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : it->indices) {
            cloud_cluster->push_back((*cloud_extracted)[idx]); //create a new point cloud for each cluster
        }
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;

        //compute + print centroid
        pcl::compute3DCentroid(*cloud_cluster, centroid);
        std::cout << centroid << std::endl;

        //broadcast
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(centroid(0), centroid(1), centroid(2)));
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "head_camera_rgb_optical_frame", "cluster_" + std::to_string(j + 1)));
        /*br.sendTransform(
                tf::StampedTransform(
                        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.020, 0.941, 1.005)), ros::Time::now(), "base_link", "head_camera_rgb_optical_frame"));*/
        //tf::TransformListener listener;

        tf::StampedTransform transform_base_link; //transform for base_link and cluster
        try {
            tf::Vector3 centroid_vec3 = transform.getOrigin(); //set centroid_vec3 to the centroid
            geometry_msgs::PointStamped base_point;
            base_point.header.frame_id = "head_camera_rgb_optical_frame";
            base_point.point.x = centroid_vec3[0]; //essentially convert the centroid to geometry_msgs::PointStamped
            base_point.point.y = centroid_vec3[1];
            base_point.point.z = centroid_vec3[2];
            geometry_msgs::PointStamped centroid_wrt_base_link; //hold the centroid point transformed with respect to base_link
            listener.transformPoint("/base_link", base_point, centroid_wrt_base_link);
            ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
                     base_point.point.x, base_point.point.y, base_point.point.z,
                     centroid_wrt_base_link.point.x, centroid_wrt_base_link.point.y, centroid_wrt_base_link.point.z, centroid_wrt_base_link.header.stamp.toSec());
            transform_base_link.setOrigin(tf::Vector3(centroid_wrt_base_link.point.x,centroid_wrt_base_link.point.y, centroid_wrt_base_link.point.z));
            transform_base_link.setRotation(tf::Quaternion(0,0,0, 1));
            br.sendTransform(tf::StampedTransform(transform_base_link, ros::Time::now(), "base_link", "cluster_" + std::to_string(j + 1)));
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        j++;
    }
}