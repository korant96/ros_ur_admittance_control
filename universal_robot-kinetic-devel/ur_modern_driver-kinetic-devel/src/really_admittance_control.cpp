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

//int i = 0;
using namespace std;
geometry_msgs::Pose target_pose1;

#define k_force 5000.0 //stiffness value
#define k_torque 3000.0 //stiffness value
#define force_threshold_value 5.0 //
#define torque_threshold_value 2.0//
#define asix_fx 1
#define asix_fy 2
#define asix_fz 3
#define asix_rx 4
#define asix_ry 5
#define asix_rz 6
#define M 1.0 //stiffness value
#define B 500.0 //stiffness value
#define K 500.0 //stiffness value
#define T 0.15 //stiffness value

#define distance 0.05
#define FORCE_CONTROL
//#define OUTPUT2FILE


float stander_fx;
float stander_fy;
float stander_fz;
float stander_tx;
float stander_ty;
float stander_tz;
float fx, fy, fz, tx, ty, tz;
int flag = 0, flag_step = 0;
string current_time;
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
		current_time = getCurrentTimeStr();
	}
#ifdef OUTPUT2FILE
	//打开输出文件
	ofstream outf("/home/ros/catkin_ws/src/universal_robot-kinetic-devel/ur_modern_driver-kinetic-devel/src/out-" + current_time + ".txt",ios::app);
	//输出到文件
	outf<< fx << ',' << fy << ',' << fz << ',' << tx <<',' << tx << ',' << ty <<',' << tz << endl;
	outf.close();
#endif
}
bool planning(moveit::planning_interface::MoveGroupInterface &arm, moveit::planning_interface::MoveGroupInterface::Plan &my_plan){
	bool success = (arm.plan(my_plan)==
	moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
	if(success){
		arm.execute(my_plan);
		return true;
	}
	return false;
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
	geometry_msgs::Pose target_pose_init;
	target_pose_init = arm.getCurrentPose().pose;
	target_pose_init.position.x = 0.317563;
	target_pose_init.position.y = 0.0827717;
	target_pose_init.position.z = 0.22652;
	target_pose_init.orientation.x = -0.0108637;
	target_pose_init.orientation.y = -0.00184182;
	target_pose_init.orientation.z = -0.000241288;
	arm.setPoseTarget(target_pose_init);
	planning(arm, my_plan);
	cout << "************init sucessed, process next program?************" << endl;
	getchar();
	sleep(1);
}

float admittance_controller(float mem_f[3], float res_f[3]){
	res_f[2] = (T * T * (mem_f[0] + 2 * mem_f[1] + mem_f[2]) - (4 * M - 2 * B + K * T * T) * res_f[0] - (-8 * M + 2 * K * T * T) * res_f[1]) / (4 * M + 2 * B + K * T * T);
	return res_f[2];
}

float force_difference_calculate(float difference, char asix, float mem_f[3], float res_f[3]){	
	float res = 0;

	if (difference > force_threshold_value ||difference < -force_threshold_value){
		switch(asix){
			case asix_fz:
				mem_f[0] = mem_f[1];
				mem_f[1] = mem_f[2];
				mem_f[2] = difference; 
				res_f[0] = res_f[1];
				res_f[1] = res_f[2];
				res = admittance_controller(mem_f, res_f);
				if(res > 0.0015)
					res = 0.0015;
				if(res < -0.0015)
					res = -0.0015;
				cout << "***************" << "Z" << ":"<< res << "***************" << endl;
				res_f[2] = res; 
				break;
			case asix_fy:
				mem_f[0] = mem_f[1];
				mem_f[1] = mem_f[2];
				mem_f[2] = difference; 
				res_f[0] = res_f[1];
				res_f[1] = res_f[2];
				res = admittance_controller(mem_f, res_f);
				if(res > 0.0015)
					res = 0.0015;
				if(res < -0.0015)
					res = -0.0015;
				cout << "***************" << "Y" << ":"<< res << "***************" << endl;	
				res_f[2] = res; 
			break;
		}
		return res;
	}
	return -0;
}

float torque_difference_calculate(float difference, char asix, float mem_f[3], float res_f[3]){
	float res = 0;

	if (difference > torque_threshold_value ||difference < -torque_threshold_value){
		switch(asix){
			case asix_rz:
				mem_f[0] = mem_f[1];
				mem_f[1] = mem_f[2];
				mem_f[2] = difference; 
				res_f[0] = res_f[1];
				res_f[1] = res_f[2];
				res = difference / k_torque;
				cout << "***************" << asix << ":"<< difference / k_torque << "***************" << endl;
				if(res > 0.2)
					res = 0.002;
				if(res < -0.2)
					res = -0.002;
				res_f[2] = res; 
				break;
				
			case asix_ry:
				mem_f[0] = mem_f[1];
				mem_f[1] = mem_f[2];
				mem_f[2] = difference; 
				res_f[0] = res_f[1];
				res_f[1] = res_f[2];
				res = difference / k_torque;
				cout << "***************" << asix << ":"<< difference / k_torque << "***************" << endl;
				if(res > 0.2)
					res = 0.002;
				if(res < -0.2)
					res = -0.002;
				res_f[2] = res; 
				break;
		}
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
	float res_fx[3] = {0}, res_fy[3] = {0}, res_fz[3] = {0};
	float res_tx[3] = {0}, res_ty[3] = {0}, res_tz[3] = {0};
	float fx_mem[3] = {0}, fy_mem[3] = {0}, fz_mem[3] = {0};
	float tx_mem[3] = {0}, ty_mem[3] = {0}, tz_mem[3] = {0};

	/****core code****/
	if(flag_step % 10 == 0){
		cout << "*********************"<< flag_step <<"*********************" <<endl;
	}
	if(difference_permition(fx_difference, fy_difference, fz_difference, tx_difference, tz_difference, tz_difference)){
		//cout << fx_difference <<','<<fy_difference<< ','<< fz_difference << "******" << tx_difference <<','<<ty_difference<< ','<< tz_difference << endl;
		arm.setGoalJointTolerance(0.000001);
		//geometry_msgs::Pose target_pose1;
		vector<double> rpy(3);      
		//target_pose1.position.x += force_difference_calculate(fz_difference, "x");
		target_pose1.position.z += force_difference_calculate(fy_difference, asix_fz, fy_mem, res_fy);
		target_pose1.position.y += force_difference_calculate(fx_difference, asix_fy, fx_mem, res_fx);
		//rpy[0] += torque_difference_calculate(tz_difference, "rx");
		rpy[1] += torque_difference_calculate(tx_difference, asix_rz, tx_mem, res_tx);
		rpy[2] -= torque_difference_calculate(ty_difference, asix_ry, ty_mem, res_ty);
		tf::Quaternion q;
		q.setRPY(rpy[0], rpy[1], rpy[2]);
		target_pose1.orientation.x = q[0];
		target_pose1.orientation.y = q[1];
		target_pose1.orientation.z = q[2];
		target_pose1.orientation.w = q[3];
		arm.setPoseTarget(target_pose1);
		planning(arm, my_plan);
	}
	else if(flag_step < 5){
		if(flag_step == 0){
			target_pose1 = arm.getCurrentPose().pose;
			cout << "***************first get***************" << endl;
		}
		//step = 0.04;
		target_pose1.position.x += 0.05;
		arm.setPoseTarget(target_pose1);
		if(planning(arm, my_plan)) flag_step++;
	}		
	else if(flag_step < ((float)distance / 0.0002) - 5){
		target_pose1.position.x += 0.0002;
		arm.setPoseTarget(target_pose1);
		if(planning(arm, my_plan)) flag_step++;
	}
	else
		cout << "*********************finished!*********************" <<endl;
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
	ros::Subscriber sub = n.subscribe("/netft_data", 100, chatterCallback_force);
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

