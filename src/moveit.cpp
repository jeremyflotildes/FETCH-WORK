//
// Created by jeremy on 7/12/21.
//

#include "moveit.h"
#include "cluster.h"

void moveGroup::graspObject() {
    ros::AsyncSpinner spinner(1);
    spinner.start();
    static const std::string PLANNING_GROUP = "arm_with_torso";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    move_group.setPlanningTime(45.0);

    const robot_state::JointModelGroup* joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("arm_with_torso");
    visual_tools.deleteAllMarkers();

    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();


    // print the name of the reference frame for this robot.
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

    // print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    // ---LOOKUP TRANSFORM BTWN CLUSTER & BASELINK---
//    tf::TransformListener listener;
    ros::Rate rate(10.0);
    try {
        moveGroup::listener.waitForTransform("/base_link", "/cluster_7", ros::Time(0), ros::Duration(4.0));
        moveGroup::listener.lookupTransform("/base_link", "/cluster_7", ros::Time(0), moveGroup::cluster_transform);
        /*addCollisionObject(planning_scene_interface);*/
        // pick(move_group);
        ROS_INFO("Goal: (%.2f, %.2f, %.2f)", cluster_transform.getOrigin().x(), cluster_transform.getOrigin().y(), cluster_transform.getOrigin().z());
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

   //code for moving eef to a particular cluster
    geometry_msgs::Pose target_pose1;
   // geometry_msgs::Pose pose_eef;
    target_pose1.orientation.x = 0.000000;
    target_pose1.orientation.y = 0.707;
    target_pose1.orientation.z = 0.000000;
    target_pose1.orientation.w = 0.707;
    
    target_pose1.position.x = moveGroup::cluster_transform.getOrigin().x();
    target_pose1.position.y = moveGroup::cluster_transform.getOrigin().y();
    target_pose1.position.z = moveGroup::cluster_transform.getOrigin().z() + 0.3;

    //moveGroup::listener.transformPose("gripper_link", target_pose1, pose_eef);

/*    target_pose1.position.x = 0.71;
    target_pose1.position.y = 0.21;
    target_pose1.position.z = 0.81;*/
    //target_pose1.header.frame_id = move_group.getEndEffectorLink();
    move_group.setPoseTarget(target_pose1);
    move_group.setGoalTolerance(0.01);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group.move();
    
    target_pose1.position.z = moveGroup::cluster_transform.getOrigin().z() + 0.01;
    move_group.setPoseTarget(target_pose1);
    move_group.move();

    closed_gripper();

    target_pose1.position.z = moveGroup::cluster_transform.getOrigin().z() + 0.3;
    move_group.setPoseTarget(target_pose1);
    move_group.move();
}

void moveGroup::lowerGripper() {

}

void moveGroup::closed_gripper() {
    // create the action client
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
        ROS_INFO("Action did not finish before the time out.");
}

void moveGroup::addCollisionObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {

/*    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(7);

    collision_objects[0].id = "cluster_7";
    collision_objects[0].header.frame_id = "gripper_link";

    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = cluster_extraction::length;
    collision_objects[0].primitives[0].dimensions[1] = cluster_extraction::width;
    collision_objects[0].primitives[0].dimensions[2] = cluster_extraction::height;

    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = moveGroup::cluster_transform.getOrigin().x();
    collision_objects[0].primitive_poses[0].position.y = moveGroup::cluster_transform.getOrigin().y();
    collision_objects[0].primitive_poses[0].position.z = moveGroup::cluster_transform.getOrigin().z();
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;

    collision_objects[0].operation = collision_objects[0].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);*/
}

 /*void moveGroup::openGripper(trajectory_msgs::JointTrajectory& posture)
{
    posture.joint_names.resize(2);
    posture.joint_names[0] = "l_gripper_finger_link";
    posture.joint_names[1] = "r_gripper_finger_link";

    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.04;
    posture.points[0].positions[1] = 0.04;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void moveGroup::closedGripper(trajectory_msgs::JointTrajectory& posture)
{
    *//* Add both finger joints of robot. *//*
    posture.joint_names.resize(2);
    posture.joint_names[0] = "l_gripper_finger_link";
    posture.joint_names[1] = "r_gripper_finger_link";

    *//* Set them as closed. *//*
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.00;
    posture.points[0].positions[1] = 0.00;
    posture.points[0].time_from_start = ros::Duration(0.5);
}


void moveGroup::pick(moveit::planning_interface::MoveGroupInterface& move_group) {

    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    grasps[0].grasp_pose.header.frame_id = "gripper_link";
    tf::Quaternion orientation;
    //orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
    orientation.setRPY(0, -1, 0); //want eef vertical pointing down
    tf::quaternionTFToMsg(orientation, grasps[0].grasp_pose.pose.orientation);
    grasps[0].grasp_pose.pose.position.x = moveGroup::cluster_transform.getOrigin().x(); //set eef pos to the relevant cluster
    grasps[0].grasp_pose.pose.position.y = moveGroup::cluster_transform.getOrigin().y();
    grasps[0].grasp_pose.pose.position.z = moveGroup::cluster_transform.getOrigin().z() + 0.5; // set eef slightly above cluster

    // Setting pre-grasp approach
    // ++++++++++++++++++++++++++
    grasps[0].pre_grasp_approach.direction.header.frame_id = "gripper_link";
    grasps[0].pre_grasp_approach.direction.vector.z = 1.0; //approach on z axis
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    // ++++++++++++++++++++++++++
    grasps[0].post_grasp_retreat.direction.header.frame_id = "gripper_link";
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    // Setting posture of eef before grasp
    // +++++++++++++++++++++++++++++++++++
    openGripper(grasps[0].pre_grasp_posture);

    // Setting posture of eef during grasp
    // +++++++++++++++++++++++++++++++++++
    closedGripper(grasps[0].grasp_posture);

    // Set support surface as table1.
   // move_group.setSupportSurfaceName("table1");
    // Call pick to pick up the object using the grasps given
    move_group.pick("cluster_1", grasps);

}
*/