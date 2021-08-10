//
// Created by jeremy on 7/8/21.
// main file that will subscribe to fetch and publish the end result after work by external classes

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <tf/transform_listener.h>

#include "../include/planar_segmentation.h"
#include "../include/crop_cloud.h"
#include "../include/cluster.h"
#include "../include/moveit.h"
#include "../include/transform_pcl.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "control_msgs/GripperCommandGoal.h"
#include "control_msgs/GripperCommandAction.h"

class subscribe_and_publish {
public:
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::NodeHandle nh;
    tf::TransformListener listener;

    crop_cloud cropCloud; //initialize class to crop the cloud
    planar_segmentation extractObj; //initialize class to extract the objects off the plane
    cluster_extraction extractClusters;
    transform_pcl transformPCL;

    bool successful_pcl = true;

    subscribe_and_publish() {
        // ---PUBLISH CROPPED CLOUD---
        pub = nh.advertise<sensor_msgs::PointCloud2>("crop", 1000);
        //pub_tf = nh.advertise<tf::Vector3>("tf", 1000);

        //--SUBSCRIBE TO FETCH---
        //while(successful_pcl = true) {
            sub = nh.subscribe("/head_camera/depth_registered/points", 1000, &subscribe_and_publish::callback, this);
        //}

    }
    void callback(sensor_msgs::PointCloud2::Ptr input_cloud) {
        while(successful_pcl == true) {
            pcl::PCLPointCloud2 output_pcl_cloud;

            transformPCL.transform(input_cloud); //transform the point cloud to base_link
            cropCloud.crop(transformPCL.transformed_cloud_pcl); //crop the input cloud from the transformed pointcloud
            extractObj.extractOffPlane(cropCloud.plane_seg_cloud); //perform planar segmentation on cropped pcl and extract objects off the plane
            extractClusters.cluster(extractObj.cloud_extracted); //pass in extracted objects for clustering

            pcl::toPCLPointCloud2(*extractObj.cloud_extracted, output_pcl_cloud); //convert pcl of extracted objects so that it can be visualized in rviz
            sensor_msgs::PointCloud2 publish_cloud; // initialize the point cloud that will be outputted/published
            pcl_conversions::fromPCL(output_pcl_cloud, publish_cloud); //convert PCL2 -> sensor_msgs point cloud to be visualized in rviz
            publish_cloud.header.frame_id = "base_link"; //need our published cloud to have parent frame base_link!
            pub.publish(publish_cloud);
            successful_pcl = false; //boolean that ensures the callback is only run once, necessary to prevent freezing
        }
    }
};

int main (int argc, char** argv) {
    ros::init(argc, argv, "crop_node");
    //ros::init(argc, argv, "grasp");

    subscribe_and_publish sapObject;
    moveGroup moveToCluster;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveToCluster.graspObject(); //call pick pipeline

    ros::waitForShutdown();
    return (0);
}