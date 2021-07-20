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

    subscribe_and_publish() {
        // ---PUBLISH CROPPED CLOUD---
        pub = nh.advertise<sensor_msgs::PointCloud2>("crop", 1000);
        //pub_tf = nh.advertise<tf::Vector3>("tf", 1000);

        //--SUBSCRIBE TO FETCH---
        sub = nh.subscribe("/head_camera/depth_registered/points", 1000, &subscribe_and_publish::callback, this);

    }
    void callback(sensor_msgs::PointCloud2::Ptr input_cloud) {
        crop_cloud cropCloud; //initialize class to crop the cloud
        planar_segmentation extractObj; //initialize class to extract the objects off the plane
        cluster_extraction extractClusters;


        pcl::PCLPointCloud2 output_pcl_cloud;
        cropCloud.crop(input_cloud); //crop the input cloud from the subscribed data
        extractObj.extractOffPlane(cropCloud.plane_seg_cloud); //perform planar segmentation on cropped pcl and extract objects off the plane
        extractClusters.cluster(extractObj.cloud_extracted); //pass in extracted objects for clustering
        pcl::toPCLPointCloud2(*extractObj.cloud_extracted, output_pcl_cloud); //convert pcl of extracted objects so that it can be visualized in rviz
        sensor_msgs::PointCloud2 publish_cloud; // initialize the point cloud that will be outputted/published
        pcl_conversions::fromPCL(output_pcl_cloud, publish_cloud); //convert PCL2 -> sensor_msgs point cloud

        pub.publish(publish_cloud);

    }
};

int main (int argc, char** argv) {
    ros::init(argc, argv, "crop_node");
    //ros::init(argc, argv, "grasp");
    subscribe_and_publish sapObject;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    moveGroup moveToCluster;

    moveToCluster.graspObject(); //call pick pipeline

    /*// create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> ac("gripper_controller/gripper_action", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    control_msgs::GripperCommandGoal goal;
    goal.command.position= 0.0;
    goal.command.max_effort= 50.0;
    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");*/
   // ros::spin();
    ros::shutdown();
    return (0);
}