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

#include "caddy_manipulation/prepare_manipulation.h"



#ifndef CROP_PC_MOVEIT_H
#define CROP_PC_MOVEIT_H
class moveGroup {
public:
    tf::TransformListener listener; //listens for transforms between base_link and clusters
    tf::TransformListener eef_listener; //listens to the end effector's position
    tf::TransformListener listener_wrist;
    tf::StampedTransform cluster_transform;
    tf::StampedTransform gripper_transform;
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::NodeHandle nh;
    tf::TransformBroadcaster br;
    bool gripped_object;
    geometry_msgs::PoseStamped jenga_stack;

    int i = 1;
    int vertical;
    bool reset;
    bool free_blocks_exist = true;

    void graspObject(); //pipeline for bringing the gripper to objects for grasping
    void closed_gripper(); //function that closes the gripper
    void open_gripper(); //function that opens gripper
    void octomap(); //creates an octomap
    void get_block_transform(int i); //gets the transform of a block to be picked
    void place(int i); //gets the place goal for a blcok
    void if_stuck(); //force fetch to reset if an error is encountered (namely goal tolerance or invalid start state)
    void callback();


    moveGroup (const std::string &move_group_name) {
        pub = nh.advertise<geometry_msgs::PointStamped>("target_pose1", 1000);
        move_group = new moveit::planning_interface::MoveGroupInterface(move_group_name);
    };

    ~moveGroup()
    {
        delete move_group;
    }

    //DECLARE the pointer
    moveit::planning_interface::MoveGroupInterface *move_group;

};
#endif //CROP_PC_MOVEIT_H
