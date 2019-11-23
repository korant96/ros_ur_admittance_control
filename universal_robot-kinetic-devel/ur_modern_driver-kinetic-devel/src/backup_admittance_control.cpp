#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "iostream"
#include <typeinfo>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <string>
int i = 0;
using namespace std;

#define k 3000.0 //stiffness value
#define force_threshold_value 5.0 //
#define torque_threshold_value 2.0//

float stander_fx;
float stander_fy;
float stander_fz;
float stander_tx;
float stander_ty;
float stander_tz;
float fx, fy, fz, tx, ty, tz;
int flag = 0, flag_step = 0;

void chatterCallback_force(const geometry_msgs::WrenchStamped::ConstPtr & msg){
    fx = msg->wrench.force.x;
    fy = msg->wrench.force.y;
    fz = msg->wrench.force.z;
    tx = msg->wrench.torque.x;
    ty = msg->wrench.torque.y;
    tz = msg->wrench.torque.z;
    if(flag == 0){//get the calibration of sensor automatically when booting
        stander_fx = fx;
        stander_fy = fy;
        stander_fz = fz;
        stander_tx = tx;
        stander_ty = ty;
        stander_tz = tz;
        flag++;
    }
    //cout<< fx <<" "<< fy <<" "<< fz <<" "<< tx <<" "<< ty <<" "<< tz << endl;
}
void addmittance_controler_inital(moveit::planning_interface::MoveGroupInterface &arm, moveit::planning_interface::MoveGroupInterface::Plan &my_plan){
    arm.setGoalJointTolerance(0.0001);
    vector <double>joint_position(6);
    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.2);
    joint_position = {-0.114552, -1.33864, 2.58024, -4.38686, -1.45662,-0.0212897};
    arm.setJointValueTarget(joint_position);
    arm.move();
    geometry_msgs::Pose target_pose1;
    target_pose1 = arm.getCurrentPose().pose;
    target_pose1.position.x = 0.317563;
    target_pose1.position.y = 0.0827717;
    target_pose1.position.z = 0.22652;
    target_pose1.orientation.x = -0.0108637;
    target_pose1.orientation.y = -0.00184182;
    target_pose1.orientation.z = -0.000241288;
    arm.setPoseTarget(target_pose1);
    bool success = (arm.plan(my_plan)==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    if(success)
        arm.execute(my_plan);
    cout << "************init sucessed************" << endl;
    sleep(1);
    return;
}
float force_difference_calculate(float difference, string asix){
  float res;
  if (difference > force_threshold_value ||difference < -force_threshold_value){
    res = difference / k;
    cout << "***************" << asix << ":"<< res << "***************" << endl;
    if(res > 0.0015)
      res = 0.0015;
    return res;
  }
  return -0;
}

float torque_difference_calculate(float difference, string asix){
  float res;
  if (difference > torque_threshold_value ||difference < -torque_threshold_value){
    cout << "***************" << asix << ":"<< difference / k << "***************" << endl;
    res = difference / k;
    if(res > 0.2)
      res = 0.002;
    return res;
  }
  return -0;
}
bool difference_permition(float fx_p, float fy_p, float fz_p, float tx_p, float ty_p, float tz_p){

  if(fx_p < -force_threshold_value || fx_p > force_threshold_value) return true;
  else if(fy_p < -force_threshold_value || fy_p > force_threshold_value) return true;
  //else if(fz_p < -force_threshold_value || fz_p > force_threshold_value) return true;
  else if(tx_p < -torque_threshold_value || tx_p > torque_threshold_value) return true;
  else if(ty_p < -torque_threshold_value || ty_p > torque_threshold_value) return true;
  //else if(tz_p < -torque_threshold_value || tz_p > torque_threshold_value) return true;
  else return false;

  //return false;
}
void addmittance_controller_running(moveit::planning_interface::MoveGroupInterface &arm, moveit::planning_interface::MoveGroupInterface::Plan &my_plan){

  double step;
  float tx_difference = tx - stander_tx, ty_difference = ty - stander_ty, tz_difference = tz - stander_tz;//torque difference between real and stander
  float fx_difference = fx - stander_fx, fy_difference = fy - stander_fy, fz_difference = fz - stander_fz;//force difference between real and stander
  //float rx = 0, ry = 0, rz = 0;

  //if(tx_difference > 1 || tx_difference < -1 || ty_difference > 1 || ty_difference < -1 || tz_difference > 1 || tz_difference < -1 ||
     //fx_difference > 2 || fx_difference < -2 || fy_difference > 2 || fy_difference < -2 || fz_difference > 2 || fz_difference < -2){
  if(difference_permition(fx_difference, fy_difference, fz_difference, tx_difference, tz_difference, tz_difference)){
    cout << fx_difference <<','<<fy_difference<< ','<< fz_difference << "******" << tx_difference <<','<<ty_difference<< ','<< tz_difference << endl;
    arm.setGoalJointTolerance(0.001);
    geometry_msgs::Pose target_pose1;
    target_pose1 = arm.getCurrentPose().pose;
    vector<double> rpy(3);
    rpy = arm.getCurrentRPY();
    //target_pose1.position.x += force_difference_calculate(fx_difference, );
    target_pose1.position.z += force_difference_calculate(fy_difference, "z");
    target_pose1.position.y -= force_difference_calculate(fx_difference, "y");
    rpy[1] += torque_difference_calculate(tx_difference, "rz");
    rpy[2] -= torque_difference_calculate(ty_difference, "ry");
    tf::Quaternion q;
    q.setRPY(rpy[0], rpy[1], rpy[2]);
    target_pose1.orientation.x = q[0];
    target_pose1.orientation.y = q[1];
    target_pose1.orientation.z = q[2];
    target_pose1.orientation.w = q[3];
    //target_pose1.orientation.x += torque_difference_calculate(tx_difference);
    //target_pose1.orientation.y += torque_difference_calculate(ty_difference);
    //target_pose1.orientation.z += torque_difference_calculate(tz_difference);
    arm.setPoseTarget(target_pose1);
    bool success = (arm.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    if(success)
        arm.execute(my_plan);
  }
  if(flag_step == 99) cout << "*********************finished!*********************" <<endl;
  else if(flag_step < 100){
      if(flag_step < 12)
        step = 0.02;
      else
        step = 0.001;
      geometry_msgs::Pose target_pose1;
      target_pose1 = arm.getCurrentPose().pose;
      target_pose1.position.x += step;
      arm.setPoseTarget(target_pose1);
      bool success = (arm.plan(my_plan)==
      moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
      //让机械臂按照规划的轨迹开始运动，对应moveit中的execute。
      if(success){
          arm.execute(my_plan);
          flag_step++;
      }
  }
  return;
}

int main(int argc, char **argv){
    cout << stander_tz << endl;
    ros::init(argc, argv, "backup_admittance_controller");
    ros::NodeHandle n;
    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    arm.setMaxAccelerationScalingFactor(0.1);
    arm.setMaxVelocityScalingFactor(0.1);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    addmittance_controler_inital(arm, my_plan);
    vector<double> rpy(3);
    rpy = arm.getCurrentRPY();
    cout << rpy[0] << ',' << rpy[1] << ',' << rpy[2] << endl;
    cout << "***************admittance controller starting***************" << endl;
    ros::Subscriber sub = n.subscribe("/netft_data", 10, chatterCallback_force);
    ros::Rate loop(20);
    while(ros::ok()){
      addmittance_controller_running(arm, my_plan);//opera addmittance controller
      loop.sleep();
    }
    ros::waitForShutdown();
    return 0;
}
