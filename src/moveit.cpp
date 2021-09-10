//
// Created by jeremy on 7/12/21.
// sends target poses for gripping

#include "moveit.h"
#include "cluster.h"

void moveGroup::graspObject() {
    ros::AsyncSpinner spinner1(1);
    spinner1.start();

    reset = true; //default state at beginning

    static const std::string PLANNING_GROUP = "arm_with_torso";

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    move_group->setPlanningTime(45.0);

    const robot_state::JointModelGroup *joint_model_group =
            move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("arm_with_torso");
    move_group->setNumPlanningAttempts(10);

    ros::Rate rate(10.0);

    get_block_transform(i); //get the transform of an object to be picked and create an octomap

    bool moving_to_above_obj = false;
    //bool third = false;

    //---FIRST GOAL--- (above obj)
    geometry_msgs::PoseStamped target_pose1;

    tf::Transform move_to_wrist_roll(tf::Quaternion(tf::createQuaternionFromRPY(0, 0, 0)), tf::Vector3(-0.366, 0, 0));
    //quaternion (165, 0 ,0) for shorter sides
    cluster_transform *= move_to_wrist_roll;

    //---set goal orientation---
    target_pose1.pose.orientation.x = cluster_transform.getRotation().x();
    target_pose1.pose.orientation.y = cluster_transform.getRotation().y();
    target_pose1.pose.orientation.z = cluster_transform.getRotation().z();
    target_pose1.pose.orientation.w = cluster_transform.getRotation().w();
    //---set goal position---
    target_pose1.pose.position.x = cluster_transform.getOrigin().x(); //position above the object
    target_pose1.pose.position.y = cluster_transform.getOrigin().y();
    target_pose1.pose.position.z = cluster_transform.getOrigin().z();

    target_pose1.header.frame_id = "/base_link";

    //---broadcast the first goal---
    tf::Transform goal_transform; // = cluster_transform;
    goal_transform.setOrigin(cluster_transform.getOrigin() + tf::Vector3(0, 0, 0.1));
    goal_transform.setRotation(cluster_transform.getRotation());
    br.sendTransform(tf::StampedTransform(goal_transform, ros::Time::now(), "base_link", "goal"));

    //---set pose target and move---
    move_group->setPoseTarget(target_pose1);
    move_group->setStartStateToCurrentState();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    /* ROS_INFO("ORIENTATION: (%.2f, %.2f, %.2f, %.2f)", target_pose1.orientation.x, target_pose1.orientation.y,
              target_pose1.orientation.z, target_pose1.orientation.w);*/
    move_group->plan(my_plan);
    visual_tools.prompt("Press next to move gripper above the object.");
    move_group->execute(my_plan);
    reset = false; //not in default position anymore

    //---SECOND GOAL--- (move to obj to pick up)
    geometry_msgs::PoseStamped target_pose2;
    open_gripper();

    //ros::spinOnce();

    target_pose2 = target_pose1; //set second goal to the first goal
    move_to_wrist_roll.setOrigin(tf::Vector3(-0.166, 0, 0));
    cluster_transform *= move_to_wrist_roll;
    target_pose2.pose.position.z = cluster_transform.getOrigin().z() - 0.365; //position end effector for gripping (gripper will slam into table @ .getOrigin().x()
    // target_pose2.header.frame_id = "/base_link";
    move_group->setPoseTarget(target_pose2);
    //move_group->setStartStateToCurrentState();

    move_group->plan(my_plan);
    visual_tools.prompt("Press next to move gripper to the object.");
    ROS_WARN("Executing second trajectory!");
    move_group->execute(my_plan);


    //---THIRD GOAL--- (grasping the obj)
    //ROS_INFO("Checking end effector orientation");
    /* ROS_INFO("SECOND GOAL: Current Orientation Y: %.5f, Target Orientation Y: %.5f", current_orientation.y(), cluster_transform.getRotation().y());
     ROS_INFO("SECOND GOAL: Current Position Z: %.5f, Target Position Z: %.5f", current_pose.pose.position.z, target_pose2.pose.position.z);*/
    visual_tools.prompt("Second goal reached, press next to close gripper.");
    ROS_WARN("Closing gripper!");
    closed_gripper();
    ROS_WARN("Gripped object!");

    //--FOURTH GOAL--- (retreat w grasped obj)
    geometry_msgs::PoseStamped target_pose3 = target_pose2;
    target_pose3.pose.position.z = cluster_transform.getOrigin().z() - 0.16; //retreat
    move_group->setPoseTarget(target_pose3);
    //move_group->setStartStateToCurrentState();
    move_group->plan(my_plan);
    visual_tools.prompt("Press next to pull the block up.");

    move_group->execute(my_plan);
    ROS_WARN("Pulling up!");

    //---FIFTH GOAL--- (place in stack)

    place(i); //pick a place goal
    move_group->setPoseTarget(jenga_stack);
    //move_group->setStartStateToCurrentState();

    move_group->plan(my_plan);
    visual_tools.prompt("Press next to move the block to stack.");
    move_group->execute(my_plan);
    ROS_INFO("Moving to stack!");

    //---SIXTH GOAL--- (open gripper and retreat)
    geometry_msgs::PoseStamped retreat_placed_block;
    visual_tools.prompt("Press next to release the block.");

    open_gripper();
    ROS_INFO("Releasing block!");

    //create retreat goal
    retreat_placed_block = jenga_stack;
    retreat_placed_block.pose.position.z = jenga_stack.pose.position.z +
                                           0.16; // +0.16 b/c relative to base_link which has up for positive z axis
    retreat_placed_block.header.frame_id = "/base_link";

    move_group->setPoseTarget(retreat_placed_block);
    //move_group->setStartStateToCurrentState();
    move_group->plan(my_plan);
    visual_tools.prompt("Press next to pull the gripper up.");
    move_group->execute(my_plan);
    ROS_INFO("Retreating!");

    visual_tools.prompt("Press next to reset the caddy.");
    //move_group->setStartStateToCurrentState();
    sleep(1.0);
    prepare_caddy_manipulation();

    //geometry_msgs::PoseStamped current_pose = move_group->getCurrentPose();
    //ROS_WARN("RETREAT GOAL: Current Position Z: %.5f, Target Position Z: %.5f", current_pose.pose.position.z, retreat_placed_block.pose.position.z);
    ROS_INFO("Resetting position!");
    reset = true;
    i++;
}

void moveGroup::closed_gripper() {

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient <control_msgs::GripperCommandAction> ac("gripper_controller/gripper_action", false);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    control_msgs::GripperCommandGoal goal;
    goal.command.position = 0.00;
    goal.command.max_effort = 60.0;
    ac.sendGoal(goal);
    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(3.0));
    gripped_object = true;

    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
        gripped_object = true;
    } else {
        ROS_INFO("Action did not finish before the time out.");
        gripped_object = true;
    }
}

void moveGroup::open_gripper() {

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient <control_msgs::GripperCommandAction> ac("gripper_controller/gripper_action", false);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    control_msgs::GripperCommandGoal goal;
    goal.command.position = 0.065;
    goal.command.max_effort = 60.0;
    ac.sendGoal(goal);
    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(6.0));
    gripped_object = false;
    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
        gripped_object = false;
    } else {
        ROS_INFO("Action did not finish before the time out.");
        gripped_object = false;
    }
}

void moveGroup::octomap() {
    ros::NodeHandle nh;
    OctomapBuilder octomap_builder(nh);
    bool playing_from_bag = false;
    bool octomap_valid = false;
    octomap_builder.clear_map();

    PointHeadClient headClient;

    std::vector <Eigen::Vector3d> poses;

    poses.push_back(Eigen::Vector3d(0.35, 0, -0.1)); //middle middle

    if (!octomap_valid) {
        octomap_builder.relatively_look_around_to_build_map(poses, true);
        auto target = cluster_transform.getOrigin();
        octomap_builder.clear_cube(cluster_transform.getOrigin().x(), cluster_transform.getOrigin().y(),
                                   cluster_transform.getOrigin().z(), 0.1, 0.05);
        octomap_valid = true;
    }
    //ros::spinOnce();
    octomap_builder.clear_cube(cluster_transform.getOrigin().x(), cluster_transform.getOrigin().y(),
                               cluster_transform.getOrigin().z(), 0.1, 0.05);
    headClient.relatively_look_at(0.7, 0, -(1.4245 - 0.8)); //look back to the center
}

void moveGroup::get_block_transform(int i) {
    try {
        listener.waitForTransform("/base_link", "cluster_" + std::to_string(i), ros::Time(0), ros::Duration(4.0));
        listener.lookupTransform("/base_link", "cluster_" + std::to_string(i), ros::Time(0), cluster_transform);
        ROS_WARN("Goal: (%.2f, %.2f, %.2f)", cluster_transform.getOrigin().x(), cluster_transform.getOrigin().y(),
                 cluster_transform.getOrigin().z());
    }
    catch (tf::TransformException ex) {
        free_blocks_exist = false;
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    octomap();
}

void moveGroup::place(int i) {

    if (i ==
        1) { //first iteration, initialize height to place the block -- rest of statements will build off of this height + position
        vertical = 1;

        jenga_stack.pose.position.x = cluster_transform.getOrigin().x();
        jenga_stack.pose.position.y = cluster_transform.getOrigin().y() - 0.2;
        jenga_stack.pose.position.z = cluster_transform.getOrigin().z() - 0.35;

        /* jenga_stack.pose.position.x = 0.67;
         jenga_stack.pose.position.y = -0.2;
         jenga_stack.pose.position.z = cluster_transform.getOrigin().z() - 0.35;*/

        //vertical orientation
        jenga_stack.pose.orientation.x = 0;
        jenga_stack.pose.orientation.y = 0.707;
        jenga_stack.pose.orientation.z = 0;
        jenga_stack.pose.orientation.w = 0.707;

        tf::Transform goal_transform(tf::Quaternion(jenga_stack.pose.orientation.x, jenga_stack.pose.orientation.y, jenga_stack.pose.orientation.z, jenga_stack.pose.orientation.w),
                                     tf::Vector3(jenga_stack.pose.position.x, jenga_stack.pose.position.y, jenga_stack.pose.position.z - 0.15));

        br.sendTransform(tf::StampedTransform(goal_transform, ros::Time::now(), "base_link", "stack_goal"));

        jenga_stack.header.frame_id = "/base_link";
        ROS_WARN("FIRST BLOCK IN VERTICAL ROW AND STACK!");
    } else if (i % 2 && vertical == 1 && i != 1) { //for first blcok on a vertical row that is not the first row
        jenga_stack.pose.position.x -= 0.04;
        jenga_stack.pose.position.y += 0.07;
        jenga_stack.pose.position.z += 0.06; //move goal up

        //vertical orientation
        jenga_stack.pose.orientation.x = 0;
        jenga_stack.pose.orientation.y = 0.707;
        jenga_stack.pose.orientation.z = 0;
        jenga_stack.pose.orientation.w = 0.707;
        ROS_WARN("FIRST BLOCK IN VERTICAL ROW!");
    } else if (i % 2 == 0 && vertical == 1) { // for second block on a vertical row
        ROS_INFO("Moving to the right.");
        jenga_stack.pose.position.y -= 0.08; //move to right
        vertical *= -1;
        ROS_WARN("SECOND BLOCK ON VERTICAL ROW!");
    } else if (i % 2 && vertical == -1) { // for first block on a horizontal row
        ROS_INFO("Moving forwards.");
        jenga_stack.pose.position.x -= 0.04; //move forwards
        jenga_stack.pose.position.y += 0.05; //move to the left slightly to put gripper over middle
        jenga_stack.pose.position.z += 0.06; //move goal up

        jenga_stack.pose.orientation.x = -0.5;
        jenga_stack.pose.orientation.y = -0.5;
        jenga_stack.pose.orientation.z = 0.5;
        jenga_stack.pose.orientation.w = -0.5;

        ROS_WARN("FIRST BLOCK ON HORIZONTAL ROW!");
    } else if (i % 2 == 0 && vertical == -1) { // for second block on a horizontal row
        ROS_INFO("Moving backwards");
        jenga_stack.pose.position.x += 0.08; //move backwards
        jenga_stack.pose.position.y -= 0.04; //move to the right slightly to put gripper over middle

        vertical *= -1;

        ROS_WARN("SECOND BLOCK ON HORIZONTAL ROW!");
    }
}
