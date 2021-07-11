//
// Created by jeremy on 7/8/21.
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <tf/transform_listener.h>

#include "../include/planar_segmentation.h"
#include "../include/crop_cloud.h"

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

        pcl::PCLPointCloud2 output_pcl_cloud;
        cropCloud.crop(input_cloud); //crop the input cloud from the subscribed data
        extractObj.extractOffPlane(cropCloud.plane_seg_cloud); //extract objects off the plane
        pcl::toPCLPointCloud2(*extractObj.cloud_extracted, output_pcl_cloud);
        sensor_msgs::PointCloud2 publish_cloud; // initialize the point cloud that will be outputted/published
        pcl_conversions::fromPCL(output_pcl_cloud, publish_cloud); //convert PCL2 -> sensor_msgs point cloud

        pub.publish(publish_cloud);
    }
};

int main (int argc, char** argv) {
    ros::init(argc, argv, "crop_node");
    subscribe_and_publish sapObject;
    ros::spin();

    return (0);
}