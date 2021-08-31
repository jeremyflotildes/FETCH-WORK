//
// Created by jeremy on 7/10/21.
// function that extracts clusters, calculates their centroids, transforms centroids with respect to base_link, and broadcasts them

#include "cluster.h"
#include "moveit.h"

//feature detection stuff
#include <vector>
#include <thread>

#include <pcl/point_types.h>
#include "tf/transform_datatypes.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "tf_conversions/tf_eigen.h"
#include <eigen_conversions/eigen_msg.h>
#include <pcl/common/pca.h>


using namespace std;
void cluster_extraction::cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_extracted) { //fixed: "undefined reference to cluster_extraction::cluster" -> function requires parent class + scope operator
    // ---CREATING THE KDTREE OBJECT FOR THE SEARCH METHOD OF THE EXTRACTION---
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_extracted);

    std::vector<pcl::PointIndices> cluster_indices; //vector containing one instance of PointIndices for each detected cluster
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 1.5 cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_extracted);
    ec.extract (cluster_indices);

    int j = 0;
    // Eigen::Vector4d centroid; --> commented out, needs to be a public variable so the moveit class can access it (from a different file) and make it a target
    //tf::TransformBroadcaster br; --> made a member variable to avoid problems with the broadcaster needing to clean up
    tf::TransformListener listener;

   /*  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(7); */

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) //iterate thru cluster_indices (each element of cluster_indices contains all the points of a cluster)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : it->indices) {
            cloud_cluster->push_back((*cloud_extracted)[idx]); //create a new point cloud for each cluster
        }
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;

        //compute + print centroid
        pcl::compute3DCentroid(*cloud_cluster, centroid);
        ROS_INFO("Centroid computed");
        //std::cout << cluster_extraction::centroid << std::endl;

        //compute orientation
        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(cloud_cluster);
        pca.setIndices(0, 0, 1, cloud_cluster->size());

        Eigen::RowVector3f major_vector = pca.getEigenVectors().col(0); //major vector indicates the dominant dimension
        //std::cout << major_vector << std::endl;

        major_vector(2) = 0; //set z to 0?
        Eigen::Quaternionf eigen_quaternionf = Eigen::Quaternionf::FromTwoVectors( Eigen::Vector3f::UnitY(), major_vector); //get quaternion from the y axis + major vector
        tf::Quaternion tf_quaternion;
        tf::quaternionEigenToTF(eigen_quaternionf.cast<double>(), tf_quaternion);

        //broadcast
        tf::Transform transform;
        //tf::Transform transform_wrist;

        transform.setOrigin(tf::Vector3(centroid(0), centroid(1), centroid(2)));
        ROS_INFO("Setting transform origin to the centroid");
        transform.setRotation(tf_quaternion);
        transform.setRotation(transform.getRotation() * tf::Quaternion(tf::Vector3(1, 0, 0), M_PI_2) * tf::Quaternion(tf::Vector3(0, 0, 1), -M_PI_2));
        ROS_INFO("Setting transform orientation");

        ros::spinOnce();
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "cluster_" + std::to_string(j+1)));
        ROS_INFO("Sending transforms!");
        //wrist_br.sendTransform(tf::StampedTransform(transform_wrist, ros::Time::now(), "base_link", "wrist_roll_link"));


        /*br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "head_camera_rgb_optical_frame", "cluster_" + std::to_string(j + 1)));

        tf::StampedTransform transform_base_link; //transform for base_link and cluster
        geometry_msgs::PointStamped centroid_wrt_base_link; //hold the centroid point transformed with respect to base_link
        try {
            tf::Vector3 centroid_vec3 = transform.getOrigin(); //set centroid_vec3 to the centroid
            geometry_msgs::PointStamped base_point;
            base_point.header.frame_id = "head_camera_rgb_optical_frame";
            base_point.point.x = centroid_vec3[0]; //essentially convert the centroid to geometry_msgs::PointStamped
            base_point.point.y = centroid_vec3[1];
            base_point.point.z = centroid_vec3[2];

            listener.waitForTransform("head_camera_rgb_optical_frame", "base_link", ros::Time(0), ros::Duration(0.5)); //give transform some time to be detected
            listener.transformPoint("/base_link", base_point, centroid_wrt_base_link);
            ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
                     base_point.point.x, base_point.point.y, base_point.point.z,
                     centroid_wrt_base_link.point.x, centroid_wrt_base_link.point.y, centroid_wrt_base_link.point.z, centroid_wrt_base_link.header.stamp.toSec());

            transform_base_link.setOrigin(tf::Vector3(centroid_wrt_base_link.point.x,centroid_wrt_base_link.point.y, centroid_wrt_base_link.point.z)); //set origin
            transform_base_link.setRotation(tf_quaternion); //set rotation

            br.sendTransform(tf::StampedTransform(transform_base_link, ros::Time::now(), "base_link", "cluster_" + std::to_string(j + 1)));
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }*/

/*        //ADD CLUSTER AS OBJECT
        collision_objects[j].header.frame_id = "gripper_link";
        collision_objects[j].id = "cluster_1"; //+ std::to_string(j + 1);

        collision_objects[j].primitives.resize(1);
        collision_objects[j].primitives[0].type = collision_objects[0].primitives[0].BOX;
        collision_objects[j].primitives[0].dimensions.resize(3);
        collision_objects[j].primitives[0].dimensions[0] = max[0] - min[0];
        collision_objects[j].primitives[0].dimensions[1] = max[1] - min[1];
        collision_objects[j].primitives[0].dimensions[2] = max[2] - min[2];

        collision_objects[j].primitive_poses.resize(1);
        collision_objects[j].primitive_poses[0].position.x = centroid_wrt_base_link.point.x;
        collision_objects[j].primitive_poses[0].position.y = centroid_wrt_base_link.point.y;
        collision_objects[j].primitive_poses[0].position.z = centroid_wrt_base_link.point.z;
        collision_objects[j].primitive_poses[0].orientation.w = 1.0;

        collision_objects[j].operation = collision_objects[j].ADD;*/

        ros::spinOnce();
        j++;
    }
    // planning_scene_interface.applyCollisionObjects(collision_objects);
}