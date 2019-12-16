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
#include<fstream>

int i = 0;
using namespace std;
geometry_msgs::Pose target_pose1;

//#define k 3000.0 //stiffness value
#define k_force 5000.0 //stiffness value
#define k_torque 3000.0 //stiffness value
#define force_threshold_value 10.0 //
#define torque_threshold_value 2.0//

#define FORCE_CONTROL


float stander_fx;
float stander_fy;
float stander_fz;
float stander_tx;
float stander_ty;
float stander_tz;
float fx, fy, fz, tx, ty, tz;
int flag = 0, flag_step = 0;

static string  getCurrentTimeStr()
{
	time_t t = time(NULL);
	char ch[64] = {0};
	strftime(ch, sizeof(ch) - 1, "%Y-%m-%d %H:%M:%S", localtime(&t));     //年-月-日 时-分-秒
	return ch;
}

/****the callback of force sensor****/
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
#ifdef OUTPUT2FILE
	//打开输出文件
	ofstream outf("/home/ros/catkin_ws/src/universal_robot-kinetic-devel/ur_modern_driver-kinetic-devel/src/out-" + current_time + ".txt",ios::app);
	//输出到文件
	outf<< fx << ',' << fy << ',' << fz << ',' << tx <<',' << tx << ',' << ty <<',' << tz << endl;
	outf.close();
#endif
}

void addmittance_controler_inital(moveit::planning_interface::MoveGroupInterface &arm, moveit::planning_interface::MoveGroupInterface::Plan &my_plan){
	flag_step = 0;
	vector <double>joint_position(6);
	arm.setGoalJointTolerance(0.0001);
	arm.setMaxAccelerationScalingFactor(1);
	arm.setMaxVelocityScalingFactor(1);
	/****The first pose***/
	joint_position = {-0.114552, -1.33864, 2.58024, -4.38686, -1.45662,-0.0212897};
	arm.setJointValueTarget(joint_position);
	arm.move();   
	geometry_msgs::Pose target_pose1;
	target_pose1 = arm.getCurrentPose().pose;
	//0.5269390.09128220.229934
	//0.0005731750.009490540.001699360.999953
	/*-0.0410188
	-1.04733
	2.00318
	-4.07847
	-1.52639
	0.000299606
	*/

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
		res = difference / k_force;
		cout << "***************" << asix << ":"<< res << "***************" << endl;
		if(res > 0.0015)
			res = 0.0015;
		if(res < -0.0015)
			res = -0.0015;

		return res;
	}
	return -0;
}

float torque_difference_calculate(float difference, string asix){
	float res;

	if (difference > torque_threshold_value ||difference < -torque_threshold_value){
		cout << "***************" << asix << ":"<< difference / k_torque << "***************" << endl;
		res = difference / k_torque;
		if(res > 0.2)
			res = 0.002;
		if(res < -0.2)
			res = -0.002;
		return res;
	}
	return -0;
}
bool difference_permition(float fx_p, float fy_p, float fz_p, float tx_p, float ty_p, float tz_p){

#ifdef FORCE_CONTROL
	if(fx_p < -force_threshold_value || fx_p > force_threshold_value) return true;
	else if(fy_p < -force_threshold_value || fy_p > force_threshold_value) return true;
	//else if(fz_p < -force_threshold_value || fz_p > force_threshold_value) return true;//not used
	else if(tx_p < -torque_threshold_value || tx_p > torque_threshold_value) return true;
	else if(ty_p < -torque_threshold_value || ty_p > torque_threshold_value) return true;
	//else if(tz_p < -torque_threshold_value || tz_p > torque_threshold_value) return true;//not used
	else return false;
#endif
#ifdef NO_FORCE_CONTROL
	return false;
#endif
}
void addmittance_controller_running(moveit::planning_interface::MoveGroupInterface &arm, moveit::planning_interface::MoveGroupInterface::Plan &my_plan){

	double step;
	float tx_difference = tx - stander_tx, ty_difference = ty - stander_ty, tz_difference = tz - stander_tz;//torque difference between real and stander
	float fx_difference = fx - stander_fx, fy_difference = fy - stander_fy, fz_difference = fz - stander_fz;//force difference between real and stander

	/****core code****/

	if(flag_step == 125) {
		cout << "*********************finished!*********************" <<endl;
	}
	if(flag_step % 10 == 0){
		cout << "*********************"<< flag_step <<"*********************" <<endl;
	}
	if(difference_permition(fx_difference, fy_difference, fz_difference, tx_difference, tz_difference, tz_difference)){
		//cout << fx_difference <<','<<fy_difference<< ','<< fz_difference << "******" << tx_difference <<','<<ty_difference<< ','<< tz_difference << endl;
		arm.setGoalJointTolerance(0.001);
		//geometry_msgs::Pose target_pose1;
		vector<double> rpy(3);      
		if(flag_step == 0){
			target_pose1 = arm.getCurrentPose().pose;
			rpy = arm.getCurrentRPY();
			cout << "***************first get***************" << endl;
		}
		//target_pose1.position.x += force_difference_calculate(fz_difference, "x");
		target_pose1.position.z += force_difference_calculate(fy_difference, "z");
		target_pose1.position.y -= force_difference_calculate(fx_difference, "y");
		//rpy[0] += torque_difference_calculate(tz_difference, "rx");
		rpy[1] += torque_difference_calculate(tx_difference, "rz");
		rpy[2] -= torque_difference_calculate(ty_difference, "ry");
		tf::Quaternion q;
		q.setRPY(rpy[0], rpy[1], rpy[2]);
		target_pose1.orientation.x = q[0];
		target_pose1.orientation.y = q[1];
		target_pose1.orientation.z = q[2];
		target_pose1.orientation.w = q[3];
		arm.setPoseTarget(target_pose1);
		bool success = (arm.plan(my_plan) ==
		moveit::planning_interface::MoveItErrorCode::SUCCESS);
		ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
		if(success)
		  //arm.execute(my_plan);
			cout << "success" << endl;
		}
		else if(flag_step < 125){
			if(flag_step < 6)
				step = 0.04;
			else
				step = 0.0005;
			//geometry_msgs::Pose target_pose1;
			if(flag_step == 0){
				target_pose1 = arm.getCurrentPose().pose;
				cout << "***************first get***************" << endl;
		}
		target_pose1.position.x += step;
		arm.setPoseTarget(target_pose1);
		bool success = (arm.plan(my_plan)==
		moveit::planning_interface::MoveItErrorCode::SUCCESS);
		ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
		//让机械臂按照规划的轨迹开始运动，对应moveit中的execute。
		if(success){
			arm.execute(my_plan);
			//cout << "success" << endl;
			flag_step++;
		}
	}
	return;
}

int main(int argc, char **argv){
	//cout << stander_tz << endl;
	ros::init(argc, argv, "admittance_controller");
	ros::NodeHandle n;   
	moveit::planning_interface::MoveGroupInterface arm("manipulator");
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	ros::AsyncSpinner spinner(2);
	spinner.start();
	addmittance_controler_inital(arm, my_plan);
	cout << "***************admittance controller starting***************" << endl;
	ros::Subscriber sub = n.subscribe("/netft_data", 10, chatterCallback_force);
	//ros::Rate loop(20);
	while(ros::ok()){
		addmittance_controller_running(arm, my_plan);//opera addmittance controller
		//target_pose1 = arm.getCurrentPose().pose;
		//ROS_INFO("hhhh");
		//loop.sleep();
	}
	ros::waitForShutdown();
	return 0;
}
