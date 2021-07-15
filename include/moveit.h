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

#ifndef CROP_PC_MOVEIT_H
#define CROP_PC_MOVEIT_H
class moveGroup {
public:
    tf::TransformListener listener;
    void graspObject();
    void openGripper(trajectory_msgs::JointTrajectory& posture);
    void pick(moveit::planning_interface::MoveGroupInterface& move_group);
    moveGroup () {

    }

};
#endif //CROP_PC_MOVEIT_H
