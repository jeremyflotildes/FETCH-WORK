//main, unorganized file -- codebase that performs cropping, planar segmentation, extraction off plane, clustering, and broadcasting tf

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <ros/ros.h>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <tf/transform_broadcaster.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

class subscribe_and_publish {
public:
    ros::Publisher pub;
    ros::Publisher pub_tf;
    ros::Subscriber sub;
    ros::NodeHandle nh;
    std::string cluster;
    tf::TransformListener listener;

    subscribe_and_publish() {
        // ---PUBLISH CROPPED CLOUD---
        pub = nh.advertise<sensor_msgs::PointCloud2>("crop", 1000);
        //pub_tf = nh.advertise<tf::Vector3>("tf", 1000);

        //--SUBSCRIBE TO FETCH---
        sub = nh.subscribe("/head_camera/depth_registered/points", 1000, &subscribe_and_publish::callback, this);

    }

    void callback(sensor_msgs::PointCloud2::Ptr input_cloud) {
        pcl::PCLPointCloud2::Ptr input_cloud_pcl (new pcl::PCLPointCloud2); //initialize PCL2 to be inputted into CropBox
        pcl_conversions::toPCL(*input_cloud, *input_cloud_pcl); //convert subscribed data (sensor_msgs/PCL2) into pcl::PCL2
        pcl::PCLPointCloud2 cropped_cloud_pcl; //initialize the PCL2 that will be outputted from CropBox

        pcl::CropBox<pcl::PCLPointCloud2> crop;
        crop.setMin(Eigen::Vector4f(-2.0, -0.18, 0.62, 1.0));
        crop.setMax(Eigen::Vector4f(2.0, 0.18, 0.92, 1.0));
        crop.setInputCloud(input_cloud_pcl);
        crop.filter(cropped_cloud_pcl);

        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_seg_cloud(new pcl::PointCloud<pcl::PointXYZ>); //initialize new PointXYZ for plane segmentation

        pcl::fromPCLPointCloud2(cropped_cloud_pcl, *plane_seg_cloud); //convert cropped point cloud into pcl::PointXYZ to be planar segmented

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
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_extracted(new pcl::PointCloud<pcl::PointXYZ>);
        //Create filtering object
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        // Extract the inliers
        extract.setInputCloud (plane_seg_cloud);
        extract.setIndices (inliers);
        extract.setNegative (true); //false -- all inliers (the plane), true -- non-inliers (off the plane)
        extract.filter (*cloud_extracted);


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
        pcl::PCLPointCloud2 output_pcl_cloud;
        pcl::toPCLPointCloud2(*cloud_extracted, output_pcl_cloud);
        sensor_msgs::PointCloud2 publish_cloud; // initialize the point cloud that will be outputted/published
        pcl_conversions::fromPCL(output_pcl_cloud, publish_cloud); //convert PCL2 -> sensor_msgs point cloud

        pub.publish(publish_cloud);

    }
    void transformPoint(const tf::TransformListener& listener) {
        //---TF LISTENER---
        while (nh.ok()) {
            tf::StampedTransform transform_base_link;
            try {
                geometry_msgs::PointStamped centroid_wrt_base_link;
                listener.waitForTransform("/base_link", "/head_camera_rgb_optical_frame",
                                          ros::Time::now(), ros::Duration(3.0));
                //listener.lookupTransform("base_link", "head_camera_rgb_optical_frame", ros::Time(0), transform);
                //listener.transformPoint("base_link", centroid, centroid_wrt_base_link);
            }
            catch (tf::TransformException ex) {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
        }
    }
};

int main (int argc, char** argv)
{
    ros::init(argc, argv, "crop_node");
    subscribe_and_publish sapObject;
    ros::spin();

    return (0);
}