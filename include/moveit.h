//
// Created by jeremy on 7/12/21.
// header file for moveit.cpp
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf/transform_datatypes.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "control_msgs/GripperCommandGoal.h"
#include "control_msgs/GripperCommandAction.h"

#include <fetch_cpp/PointHeadClient.h>
#include "common_perception/OctomapBuilder.h"


#ifndef CROP_PC_MOVEIT_H
#define CROP_PC_MOVEIT_H
class moveGroup {
public:
    tf::TransformListener listener; //listens for transforms between base_link and clusters
    tf::TransformListener eef_listener; //listens to the end effector's position
    tf::TransformListener listener_wrist;
    tf::StampedTransform cluster_transform;
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::NodeHandle nh;
    tf::TransformBroadcaster br;
    bool gripped_object;

    void graspObject(); //pipeline for bringing the gripper to objects for grasping
    void closed_gripper(); //function that closes the gripper
    void open_gripper();
    void octomap();
    void callback();

    //functions from the pick and place pipeline from the moveit docs, likely unnecessary
    /*void openGripper();
    void closedGripper(trajectory_msgs::JointTrajectory& posture);
    void pick(moveit::planning_interface::MoveGroupInterface& move_group);
    void addCollisionObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);*/

    moveGroup () {
        pub = nh.advertise<geometry_msgs::PointStamped>("target_pose1", 1000);
    }

};
#endif //CROP_PC_MOVEIT_H
