//
// Created by jeremy on 7/12/21.
//
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

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "control_msgs/GripperCommandGoal.h"
#include "control_msgs/GripperCommandAction.h"

#ifndef CROP_PC_MOVEIT_H
#define CROP_PC_MOVEIT_H
class moveGroup {
public:
    tf::TransformListener listener;
    tf::StampedTransform cluster_transform;
    ros::Publisher pub;
    void graspObject();
    void openGripper(trajectory_msgs::JointTrajectory& posture);
    void closedGripper(trajectory_msgs::JointTrajectory& posture);
    void addCollisionObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);
    void pick(moveit::planning_interface::MoveGroupInterface& move_group);
    void closed_gripper();
    void lowerGripper();
    moveGroup () {

    }

};
#endif //CROP_PC_MOVEIT_H
