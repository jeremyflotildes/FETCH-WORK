//
// Created by jeremy on 7/12/21.
// sends target poses for gripping

#include "moveit.h"
#include "cluster.h"

void moveGroup::graspObject() {
    ros::AsyncSpinner spinner1(1);
    spinner1.start();

    static const std::string PLANNING_GROUP = "arm_with_torso";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    move_group.setPlanningTime(45.0);

    const robot_state::JointModelGroup* joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    
/*    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("arm_with_torso");
    visual_tools.deleteAllMarkers();

    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");*/

    // ---LOOKUP TRANSFORM BTWN CLUSTER & BASELINK---
//    tf::TransformListener listener;
    ros::Rate rate(10.0);
    tf::StampedTransform eef_transform;
    tf::StampedTransform wrist_transform;
    try {
        listener.waitForTransform("/base_link", "/cluster_6", ros::Time(0), ros::Duration(4.0));
        /*listener_wrist.waitForTransform("/base_link", "/wrist_roll_link", ros::Time(0), ros::Duration(4.0));*/

        listener.lookupTransform("/base_link", "/cluster_6", ros::Time(0), cluster_transform);
/*        listener_wrist.lookupTransform("/base_link", "/wrist_roll_link", ros::Time(0), wrist_transform);*/
        /*addCollisionObject(planning_scene_interface);*/
        // pick(move_group);
        ROS_INFO("Goal: (%.2f, %.2f, %.2f)", cluster_transform.getOrigin().x(), cluster_transform.getOrigin().y(), cluster_transform.getOrigin().z());
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    octomap();

    bool second = false;
    bool third = false;

    //---FIRST GOAL---
    //geometry_msgs::PoseStamped transformed_tp;
    geometry_msgs::PoseStamped target_pose1;

/*    //old goal, uses quaternions to point gripper downwards
    target_pose1.orientation.x = 0;
    target_pose1.orientation.y = 0.707;
    target_pose1.orientation.z = moveGroup::cluster_transform.getRotation().z();
    target_pose1.orientation.w = 0.707;*/

    //flip orientation so it is more ideal for fetch's gripper
    //cluster_transform.setRotation(cluster_transform.getRotation() * tf::Quaternion(tf::Vector3(1, 0, 0), M_PI_2) * tf::Quaternion(tf::Vector3(0, 0, 1), -M_PI_2));

    //tf::createQuaternion(90, 0, 0);

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
    //transformed_tp.header.frame_id = "/wrist_roll_link";

    //ROS_INFO("transformed_tp pos: (%.2f, %.2f, %.2f, %.2f):", target_pose1.pose.orientation.x, target_pose1.pose.orientation.y, target_pose1.pose.orientation.z, target_pose1.pose.orientation.w);
    //ROS_INFO("transformed_tp rot: X: " << transformed_tp.pose.orientation.x << " Y: " << transformed_tp.pose.orientation.y << " z: " << transformed_tp.pose.orientation.z << " w: " << transformed_tp.pose.orientation.w);

    //tf::Quaternion pose_orientation(transformed_tp.getRotation().x(), transformed_tp.getRotation().y(), transformed_tp.getRotation().z(), transformed_tp.getRotation().w());
    //pose_orientation = pose_orientation.normalize();

    //---broadcast the first goal---, should be able to visualize in rviz, if not -- try publishing and subscribing to the pose
    tf::Transform goal_transform; // = cluster_transform;
    goal_transform.setOrigin(cluster_transform.getOrigin() + tf::Vector3(0, 0, 0.1));
    goal_transform.setRotation(cluster_transform.getRotation());
    br.sendTransform(tf::StampedTransform(goal_transform, ros::Time::now(), "base_link", "goal"));

   //publishing pose
   //moveGroup::pub.publish(target_pose1);

    move_group.setPoseTarget(target_pose1);
   /* ROS_INFO("ORIENTATION: (%.2f, %.2f, %.2f, %.2f)", target_pose1.orientation.x, target_pose1.orientation.y,
             target_pose1.orientation.z, target_pose1.orientation.w);*/
    move_group.move(); //move robot
    second = true;
    //moveit::planning_interface::move_group::move();

    //---SECOND GOAL---
    geometry_msgs::PoseStamped target_pose2;
    geometry_msgs::PoseStamped current_pose_1;
    open_gripper();
    while(second == true) {
        current_pose_1 = move_group.getCurrentPose();
        ROS_INFO("Current Orientation X: %.5f, Target Orientation X: %.5f", current_pose_1.pose.orientation.x, target_pose1.pose.orientation.x);
        ROS_INFO("Current Position X: %.5f, Target Position X: %.5f", current_pose_1.pose.position.x, target_pose1.pose.position.x);
        if ((abs(abs(current_pose_1.pose.orientation.x) - abs(target_pose1.pose.orientation.x)) < 0.002) && (abs(current_pose_1.pose.position.x - target_pose1.pose.position.x) < 0.002) && (second == true)) {
            ROS_INFO("Target 1 reached, waiting 4 seconds...");
            sleep(4.0);
            ROS_INFO("Executing second trajectory!");
            target_pose2 = target_pose1; //set second goal to the first goal
            move_to_wrist_roll.setOrigin(tf::Vector3(-0.166, 0, 0));
            cluster_transform *= move_to_wrist_roll;
            target_pose2.pose.position.z = cluster_transform.getOrigin().z() - 0.35; //position end effector for gripping (gripper will slam into table @ .getOrigin().x()
            // target_pose2.header.frame_id = "/base_link";
            move_group.setPoseTarget(target_pose2);
            move_group.move();
            second = false;
        }
    }

    gripped_object = false;
    while(gripped_object == false && second == false) {
        geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
        eef_listener.lookupTransform("/base_link", "gripper_link", ros::Time(0), eef_transform);
        tf::Quaternion current_orientation = eef_transform.getRotation();
        //ROS_INFO("Checking end effector orientation");
        if((abs(current_orientation.y() - cluster_transform.getRotation().y()) < 0.001)
        && ((abs(current_pose.pose.position.z - target_pose2.pose.position.z)) < 0.015)) {
            ROS_INFO("Target 2 reached, waiting 2 seconds...");
            sleep(2.0);
            ROS_INFO("Closing gripper!");
            closed_gripper();
        }
    }
    geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
    //check if eef is in proximity of goal and then close gripper (.move() won't block for some reason?)
   gripped_object = false;
   while(gripped_object == false) {
        eef_listener.lookupTransform("/base_link", "gripper_link", ros::Time(0), eef_transform);
        tf::Quaternion current_orientation = eef_transform.getRotation();
        //ROS_INFO("Checking end effector orientation");
        if((abs(current_orientation.y() - cluster_transform.getRotation().y()) < 0.001)  && ((abs(current_pose.pose.position.y - cluster_transform.getOrigin().y())) < 0.015)) {
            closed_gripper();
            ROS_INFO("Pointing down!");
        }
    }

   //--THIRD GOAL---
   if(gripped_object == true) {
       geometry_msgs::PoseStamped target_pose3 = target_pose2;
       target_pose3.pose.position.z = cluster_transform.getOrigin().z(); //retreat
       //target_pose3.header.frame_id = "/base_link";
       move_group.setPoseTarget(target_pose3);
       move_group.move();
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
    bool finished_before_timeout = ac.waitForResult(ros::Duration(6.0));
    gripped_object = true;

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
        moveGroup::gripped_object = true;
    }
    else {
        ROS_INFO("Action did not finish before the time out.");
        moveGroup::gripped_object = true;
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
    goal.command.position= 0.10;
    goal.command.max_effort= 60.0;
    ac.sendGoal(goal);
    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(6.0));
    /* if (finished_before_timeout)
     {
         actionlib::SimpleClientGoalState state = ac.getState();
         ROS_INFO("Action finished: %s",state.toString().c_str());
         moveGroup::gripped_object = true;
     }
     else {
         ROS_INFO("Action did not finish before the time out.");
         moveGroup::gripped_object = true;
     }*/
}

void moveGroup::octomap() {
    ros::NodeHandle nh;
    OctomapBuilder octomap_builder(nh);
    bool playing_from_bag = false;
    bool octomap_valid = false;
    octomap_builder.clear_map();

    PointHeadClient headClient;

    std::vector<Eigen::Vector3d> poses;

/*
    poses.push_back(Eigen::Vector3d(0.7, 0.5, 0.5 )); //top left
    poses.push_back(Eigen::Vector3d(0.7, 0 , 0.5)); //top middle
    poses.push_back(Eigen::Vector3d(0.7, -0.5, 0.5 )); //top right
*/

    poses.push_back(Eigen::Vector3d(0.7, -0.5, 0.1)); //middle right
    poses.push_back(Eigen::Vector3d(0.7, 0 , 0.1)); //middle middle
    poses.push_back(Eigen::Vector3d(0.7, 0.5, 0.1)); //middle left

  /*  poses.push_back(Eigen::Vector3d(0.7, 0.5, -1.5 )); //bottom left
    poses.push_back(Eigen::Vector3d(0.7, 0 ,-1.5 )); //bottom mid
    poses.push_back(Eigen::Vector3d(0.7, -0.5,-1.5 )); // bottom right*/

    if(!octomap_valid) {
        octomap_builder.relatively_look_around_to_build_map(poses, true);
        auto target = cluster_transform.getOrigin();
        //octomap_builder.clear_cube(target.x, target.y, target.z, side_length, side_length / 2);//clear the size of the object from the octomap. Should maybe just be the size of the gripper instead
        octomap_builder.clear_cube(cluster_transform.getOrigin().x(), cluster_transform.getOrigin().y(), cluster_transform.getOrigin().z(), 0.1, 0.04);
        //box_filter.setRotation(0,0,0); any way to set rotation?
        octomap_valid = true;
    }
    headClient.relatively_look_at(0.7, 0, -(1.4245 - 0.8) ); //look back to the center
}


//void moveGroup::addCollisionObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {

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
//}

 /*void moveGroup::openGripper(trajectory_msgs::JointTrajectory& posture)
{
   // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> ac("gripper_controller/gripper_action", false);

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