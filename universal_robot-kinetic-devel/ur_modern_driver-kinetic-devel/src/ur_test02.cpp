#include<moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <vector>
using namespace std;
int test1(moveit::planning_interface::MoveGroupInterface &arm, moveit::planning_interface::MoveGroupInterface::Plan &my_plan){
    /*
    std::cout<< arm.getCurrentPose().pose.position.x << std::endl;
    std::cout<< arm.getCurrentPose().pose.position.y << std::endl;
    std::cout<< arm.getCurrentPose().pose.position.z << std::endl;
    */
    vector <double>ii(6,0);
    //std::cout << "sucess" << std::endl;
    //arm.setGoalJointTolerance(0.01);
    //robot_state::RobotState start_state(*arm.getCurrentState());
    geometry_msgs::Pose target_pose1;
    cout << "arm.getGoalJointTolerance() = " << arm.getGoalJointTolerance() << endl;
    //bool flag;
    ii =  arm.getCurrentJointValues();
    for(int i = 0; i < 6;i++)
        cout << ii[i]<< endl;
    std::cout << arm.getCurrentPose().pose.position.x << arm.getCurrentPose().pose.position.y << arm.getCurrentPose().pose.position.z << std::endl;
    std::cout << arm.getCurrentPose().pose.orientation.x << arm.getCurrentPose().pose.orientation.y << arm.getCurrentPose().pose.orientation.z<< arm.getCurrentPose().pose.orientation.w<< std::endl;
    /*
    target_pose1.orientation.x= arm.getCurrentPose().pose.orientation.x;
    target_pose1.orientation.y = arm.getCurrentPose().pose.orientation.y;
    target_pose1.orientation.z = arm.getCurrentPose().pose.orientation.z;
    target_pose1.orientation.w = arm.getCurrentPose().pose.orientation.w;
    target_pose1.position.x = arm.getCurrentPose().pose.position.x-0.02;
    target_pose1.position.y = arm.getCurrentPose().pose.position.y;
    target_pose1.position.z = arm.getCurrentPose().pose.position.z-0.02;
    */
/*
    cout << "input manual?";
    if(cin >> flag){
      cout << "position x = ";
      cin >> target_pose1.position.x ;
      cout << "position y = ";
      cin >> target_pose1.position.y;
      cout << "position z = ";
      cin >> target_pose1.position.z;
      cout << "orientation x = ";
      cin >> target_pose1.orientation.x;
      cout << "orientation y = ";
      cin >> target_pose1.orientation.y;
      cout << "orientation z = ";
      cin >> target_pose1.orientation.z;

    }*/
/*
    arm.setPoseTarget(target_pose1);
    bool success = (arm.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    //让机械臂按照规划的轨迹开始运动，对应moveit中的execute。
    if(success)
        arm.execute(my_plan);
*/
    return -0;
}

int armgo(moveit::planning_interface::MoveGroupInterface &arm, moveit::planning_interface::MoveGroupInterface::Plan &my_plan){

    //arm.setGoalJointTolerance(0.1);
    geometry_msgs::Pose target_pose1;
    //ros::Rate loop(1);
    //int flag = 0;
    ros::Rate loop(1);
    while(ros::ok()){
      //arm.setGoalJointTolerance(0.001);
      target_pose1 = arm.getCurrentPose().pose;
      target_pose1.position.x += 0.001;
      arm.setPoseTarget(target_pose1);
      bool success = (arm.plan(my_plan)==
      moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
      //让机械臂按照规划的轨迹开始运动，对应moveit中的execute。
      if(success)
          arm.execute(my_plan);
      loop.sleep();
      }
    return -0;
}
int armgo_once(moveit::planning_interface::MoveGroupInterface &arm, moveit::planning_interface::MoveGroupInterface::Plan &my_plan){
   std::cout << "go once" << std::endl;
   //arm.setGoalJointTolerance(0.0001);

//   move_group.setPoseTarget(target_pose1);
//   move_group.setPlanningTime(10.0);
//   success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//   ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

    geometry_msgs::Pose target_pose1;
    target_pose1 = arm.getCurrentPose().pose;
    target_pose1.position.x = 0.317563;
    target_pose1.position.y = 0.0827717;
    target_pose1.position.z = 0.22652;

    target_pose1.orientation.x = -0.0108637;
    target_pose1.orientation.y = -0.00184182;
    target_pose1.orientation.z = -0.000241288;
    target_pose1.orientation.w = 0.999944;
    arm.setPoseTarget(target_pose1);
    bool success = (arm.plan(my_plan)==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    if(success)
        arm.execute(my_plan);
  return -0;

}
int armgo_once2(moveit::planning_interface::MoveGroupInterface &arm, moveit::planning_interface::MoveGroupInterface::Plan &my_plan){
  std::cout << "go once2" << std::endl;
  arm.setGoalJointTolerance(0.0001);
  moveit_msgs::OrientationConstraint ocm;
  //const robot_state::JointModelGroup *joint_model_group =
      //arm.getCurrentState()->getJointModel("arm");
  robot_state::RobotState start_state(*arm.getCurrentState());
  const robot_state::JointModelGroup *joint_model_group=start_state.getJointModelGroup(arm.getName());
  std::vector<double> joint_group_positions;
  moveit::core::RobotStatePtr current_state = arm.getCurrentState();
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  /*
  ocm.link_name = "Joint6";
  ocm.header.frame_id = "base_link";
  ocm.orientation.x = 0;
  ocm.orientation.y = 0;
  ocm.orientation.z = 0;
  ocm.orientation.w = 1;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  arm.setPathConstraints(test_constraints);
  //robot_state::RobotState start_state(*arm.getCurrentState());
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation.x = 0.317563;
  start_pose2.orientation.y =  0.0827717;
  start_pose2.orientation.z = 0.22652;
  //start_pose2.orientation.w = 0.0;
  start_pose2.position.x = -0.0108637;
  start_pose2.position.y = -0.00184182;
  start_pose2.position.z = -0.000241288;
  start_pose2.orientation.w = 0.999944;
  start_state.setFromIK(joint_model_group, start_pose2);
  arm.setStartState(start_state);
  */
//   move_group.setPoseTarget(target_pose1);
//   move_group.setPlanningTime(10.0);
//   success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//   ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

   geometry_msgs::Pose target_pose1;
   target_pose1 = arm.getCurrentPose().pose;
   target_pose1.position.x = 0.717563;
   target_pose1.position.y = 0.0827717;
   target_pose1.position.z = 0.22652;

   target_pose1.orientation.x = -0.0108637;
   target_pose1.orientation.y = -0.00184182;
   target_pose1.orientation.z = -0.000241288;
   target_pose1.orientation.w = 0.999944;
   arm.setPoseTarget(target_pose1);
   bool success = (arm.plan(my_plan)==
   moveit::planning_interface::MoveItErrorCode::SUCCESS);
   ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
   if(success)
       arm.execute(my_plan);
 return -0;
}
int main(int argc, char **argv)
{
    char flag;
  //初始化，其中ur_test02为节点名
    ros::init(argc, argv, "ur_test02");
    //多线程
    ros::AsyncSpinner spinner(1);
    //开启新的线
    spinner.start();
 
    //初始化需要使用move group控制的机械臂中的arm group
    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    arm.setGoalJointTolerance(0.00000001);
    test1(arm, my_plan);
      vector <double>joint_position(6);
      //arm.setMaxAccelerationScalingFactor(0.1);
      //arm.setMaxVelocityScalingFactor(0.1);
      joint_position = {-0.114552, -1.33864, 2.58024, -4.38686, -1.45662,-0.0212897};
      arm.setJointValueTarget(joint_position);
      //test1(arm, my_plan);
      arm.move();
      std::cout << "sucess" << std::endl;
      sleep(1);
      //armgo_once(arm, my_plan);
      //armgo_once2(arm, my_plan);
      test1(arm, my_plan);
    cin >> flag;
    if(flag == 'g'){
      armgo(arm, my_plan);
    }
    ros::shutdown();
    return 0;
}
