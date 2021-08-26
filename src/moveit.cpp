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

    const robot_state::JointModelGroup* joint_model_group =
            move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
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
   /* ROS_INFO("ORIENTATION: (%.2f, %.2f, %.2f, %.2f)", target_pose1.orientation.x, target_pose1.orientation.y,
             target_pose1.orientation.z, target_pose1.orientation.w);*/
    move_group->move(); //move robot
    reset = false; //not in default position anymore
    moving_to_above_obj = true;

    //---SECOND GOAL--- (move to obj to pick up)
    geometry_msgs::PoseStamped target_pose2;
    open_gripper();
    while(moving_to_above_obj == true) { //as the gripper moves to first target
        geometry_msgs::PoseStamped current_pose = move_group->getCurrentPose();
    /*    ROS_INFO("FIRST GOAL: Current Orientation X: %.5f, Target Orientation X: %.5f", current_pose_1.pose.orientation.x, target_pose1.pose.orientation.x);
        ROS_INFO("FIRST GOAL: Current Position X: %.5f, Target Position X: %.5f", current_pose_1.pose.position.x, target_pose1.pose.position.x);*/
        if ((abs(abs(current_pose.pose.orientation.x) - abs(target_pose1.pose.orientation.x)) < 0.002)
        && (abs(current_pose.pose.position.x - target_pose1.pose.position.x) < 0.002) && (moving_to_above_obj == true)) { //only approach obj when position and orientation are within the goal above the obj
            ROS_WARN("First goal reached, waiting 4 seconds...");
            sleep(4.0);
            ROS_WARN("Executing second trajectory!");
            target_pose2 = target_pose1; //set second goal to the first goal
            move_to_wrist_roll.setOrigin(tf::Vector3(-0.166, 0, 0));
            cluster_transform *= move_to_wrist_roll;
            target_pose2.pose.position.z = cluster_transform.getOrigin().z() - 0.365; //position end effector for gripping (gripper will slam into table @ .getOrigin().x()
            // target_pose2.header.frame_id = "/base_link";
            move_group->setPoseTarget(target_pose2);
            move_group->move();
            moving_to_above_obj = false; //has completed move to above obj
        }
    }

    //---THIRD GOAL--- (grasping the obj)
    tf::StampedTransform eef_transform; //transform of the gripper
    gripped_object = false;
    while(gripped_object == false && moving_to_above_obj == false) { //only run once moving_to_above_obj == false, aka after goal above obj is reached
        geometry_msgs::PoseStamped current_pose = move_group->getCurrentPose();
        eef_listener.lookupTransform("/base_link", "gripper_link", ros::Time(0), eef_transform);
        tf::Quaternion current_orientation = eef_transform.getRotation();
        //ROS_INFO("Checking end effector orientation");
       /* ROS_INFO("SECOND GOAL: Current Orientation Y: %.5f, Target Orientation Y: %.5f", current_orientation.y(), cluster_transform.getRotation().y());
        ROS_INFO("SECOND GOAL: Current Position Z: %.5f, Target Position Z: %.5f", current_pose.pose.position.z, target_pose2.pose.position.z);*/

        if((abs(abs(current_orientation.y()) - abs(cluster_transform.getRotation().y())) < 0.001)
            && ((abs(current_pose.pose.position.z - target_pose2.pose.position.z)) < 0.015)) { //when gripper is in proper proximity of the obj
            ROS_WARN("Second Goal reached, waiting 2 seconds...");
            sleep(2.0);
            ROS_WARN("Closing gripper!");
            closed_gripper();
            ROS_WARN("Gripped object!");
        }
    }

    bool retreated_from_picked_object;
    bool retreated_from_placed_object;
   //--FOURTH GOAL--- (retreat w grasped obj)
   geometry_msgs::PoseStamped target_pose3 = target_pose2;
   if(gripped_object == true) {
       ROS_WARN("Pulling up!");
       target_pose3.pose.position.z = cluster_transform.getOrigin().z() - 0.16; //retreat
       move_group->setPoseTarget(target_pose3);
       move_group->move();
       retreated_from_picked_object = false;
   }

   //---FIFTH GOAL--- (place in stack)
   while(retreated_from_picked_object == false) {
       geometry_msgs::PoseStamped current_pose = move_group->getCurrentPose();
       //ROS_WARN("PICK GOAL: Current Position Z: %.5f, Target Position Z: %.5f", current_pose.pose.position.z, target_pose3.pose.position.z);
       if (abs(abs(current_pose.pose.position.z) - abs(target_pose3.pose.position.z)) < 0.0015) { //when gripper has fully retreated
           retreated_from_picked_object = true; //when curr pos == retreat goal pos

           place(i); //pick a place goal
           move_group->setPoseTarget(jenga_stack);
           ROS_INFO("Moving to stack!");
           move_group->move();

           retreated_from_placed_object = false;
       }
   }

   //---SIXTH GOAL--- (open gripper and retreat)
   bool placed_object = false;
   geometry_msgs::PoseStamped retreat_placed_block;
   while(retreated_from_placed_object == false) {
       geometry_msgs::PoseStamped current_pose = move_group->getCurrentPose();
       //ROS_WARN("PLACE GOAL: Current Position Y: %.5f, Target Position Y: %.5f", current_pose.pose.position.y, jenga_stack.pose.position.y);
       if ((abs(abs(current_pose.pose.position.y) - abs(jenga_stack.pose.position.y)) < 0.0015
       && (abs(abs(current_pose.pose.position.z) - abs(jenga_stack.pose.position.z)) < 0.0015)
       && placed_object == false)) { //when gripper has reached place goal

           open_gripper();
           ROS_INFO("Releasing block!");
           placed_object = true;

           //create retreat goal
           retreat_placed_block = jenga_stack;
           retreat_placed_block.pose.position.z = jenga_stack.pose.position.z + 0.16; // +0.16 b/c relative to base_link which has up for positive z axis
           retreat_placed_block.header.frame_id = "/base_link";

           move_group->setPoseTarget(retreat_placed_block);
           move_group->move();
           ROS_INFO("Retreating!");
       }

       if(abs(abs(current_pose.pose.position.z) - abs(retreat_placed_block.pose.position.z)) < 0.001) { //when gripper has reached retreat goal
           retreated_from_placed_object = true;
           move_group->setStartStateToCurrentState();
           sleep(1.0);
           prepare_caddy_manipulation();
       }
   }

   while(retreated_from_placed_object == true && reset == false) {
       geometry_msgs::PoseStamped current_pose = move_group->getCurrentPose();
       //ROS_WARN("RETREAT GOAL: Current Position Z: %.5f, Target Position Z: %.5f", current_pose.pose.position.z, retreat_placed_block.pose.position.z);
       listener.waitForTransform("/base_link", "/gripper_link", ros::Time(0), ros::Duration(4.0));
       listener.lookupTransform("/base_link", "/gripper_link", ros::Time(0), gripper_transform);
       ROS_INFO("Resetting position!");
       if (abs(abs(gripper_transform.getOrigin().z()) - 1.772) < 0.0015) { //check if gripper has retreated properly
           sleep(2);
           reset = true;
       }
   }
   i++;
}

void moveGroup::if_stuck() { //function that detects when robot is stuck or what not and forces a reset
    // common in my simulations -- "Invalid Trajectory: start point deviates from current robot state more than 0.01"
    bool success = (move_group->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success) {
        ROS_INFO("ARM STUCK.");
        prepare_caddy_manipulation();
    }
}

void moveGroup::closed_gripper() {

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> ac("gripper_controller/gripper_action", false);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    control_msgs::GripperCommandGoal goal;
    goal.command.position= 0.00;
    goal.command.max_effort= 60.0;
    ac.sendGoal(goal);
    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(3.0));
    gripped_object = true;

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
       gripped_object = true;
    }
    else {
        ROS_INFO("Action did not finish before the time out.");
        gripped_object = true;
    }
}

void moveGroup::open_gripper() {

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> ac("gripper_controller/gripper_action", false);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    control_msgs::GripperCommandGoal goal;
    goal.command.position= 0.065;
    goal.command.max_effort= 60.0;
    ac.sendGoal(goal);
    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(6.0));
    gripped_object = false;
    if (finished_before_timeout)
     {
         actionlib::SimpleClientGoalState state = ac.getState();
         ROS_INFO("Action finished: %s",state.toString().c_str());
         gripped_object = false;
     }
     else {
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

    std::vector<Eigen::Vector3d> poses;

    poses.push_back(Eigen::Vector3d(0.35, 0 , -0.1)); //middle middle

    if(!octomap_valid) {
        octomap_builder.relatively_look_around_to_build_map(poses, true);
        auto target = cluster_transform.getOrigin();
        octomap_builder.clear_cube(cluster_transform.getOrigin().x(), cluster_transform.getOrigin().y(), cluster_transform.getOrigin().z(), 0.1, 0.05);
        octomap_valid = true;
    }
    octomap_builder.clear_cube(cluster_transform.getOrigin().x(), cluster_transform.getOrigin().y(), cluster_transform.getOrigin().z(), 0.1, 0.05);
    headClient.relatively_look_at(0.7, 0, -(1.4245 - 0.8) ); //look back to the center
}

void moveGroup::get_block_transform(int i) {
    try {
            listener.waitForTransform("/base_link", "cluster_" + std::to_string(i), ros::Time(0), ros::Duration(4.0));
            listener.lookupTransform("/base_link", "cluster_" + std::to_string(i), ros::Time(0), cluster_transform);
            ROS_WARN("Goal: (%.2f, %.2f, %.2f)", cluster_transform.getOrigin().x(), cluster_transform.getOrigin().y(), cluster_transform.getOrigin().z());
    }
    catch (tf::TransformException ex) {
            free_blocks_exist = false;
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
    }
    octomap();
}
void moveGroup::place(int i) {

    if(i == 1){ //first iteration, initialize height to place the block -- rest of statements will build off of this height + position
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

        tf::Transform goal_transform(tf::Quaternion(0, 0.707, 0, 0.707), tf::Vector3(0.67, -0.2, 1.0));

        br.sendTransform(tf::StampedTransform(goal_transform, ros::Time::now(), "base_link", "stack_goal"));

        jenga_stack.header.frame_id = "/base_link";
        ROS_WARN("FIRST BLOCK IN VERTICAL ROW AND STACK!");
    }

    else if(i % 2 && vertical == 1 && i != 1) { //for first blcok on a vertical row that is not the first row
        jenga_stack.pose.position.x -= 0.04;
        jenga_stack.pose.position.y += 0.07;
        jenga_stack.pose.position.z += 0.06; //move goal up

        //vertical orientation
        jenga_stack.pose.orientation.x = 0;
        jenga_stack.pose.orientation.y = 0.707;
        jenga_stack.pose.orientation.z = 0;
        jenga_stack.pose.orientation.w = 0.707;
        ROS_WARN("FIRST BLOCK IN VERTICAL ROW!");
    }

    else if (i % 2 == 0 && vertical == 1) { // for second block on a vertical row
        ROS_INFO("Moving to the right.");
        jenga_stack.pose.position.y -= 0.08; //move to right
        vertical *= -1;
        ROS_WARN("SECOND BLOCK ON VERTICAL ROW!");
    }

    else if (i % 2 && vertical == -1) { // for first block on a horizontal row
        ROS_INFO("Moving forwards.");
        jenga_stack.pose.position.x -= 0.04; //move forwards
        jenga_stack.pose.position.y += 0.05; //move to the left slightly to put gripper over middle
        jenga_stack.pose.position.z += 0.06; //move goal up

       jenga_stack.pose.orientation.x = -0.5;
       jenga_stack.pose.orientation.y = -0.5;
       jenga_stack.pose.orientation.z = 0.5;
       jenga_stack.pose.orientation.w = -0.5;

       ROS_WARN("FIRST BLOCK ON HORIZONTAL ROW!");
    }

    else if (i % 2 == 0 && vertical == -1) { // for second block on a horizontal row
        ROS_INFO("Moving backwards");
        jenga_stack.pose.position.x += 0.08; //move backwards
        jenga_stack.pose.position.y -= 0.04; //move to the right slightly to put gripper over middle

        vertical *= -1;

        ROS_WARN("SECOND BLOCK ON HORIZONTAL ROW!");
    }
}
