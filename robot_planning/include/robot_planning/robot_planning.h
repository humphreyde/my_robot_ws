#ifndef ROBOT_PLANNING_H_INCLUDED
#define ROBOT_PLANNING_H_INCLUDED

/**************ros头文件*******************/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

#include <geometry_msgs/Pose.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>

#include "baxter_core_msgs/JointCommand.h"
#include <gazebo_msgs/LinkState.h>
//#include <octomap/octomap.h>
#include <control_msgs/JointControllerState.h>
#include <tf/transform_listener.h>


/**************C++常用头*******************/
#include <vector>
#include <stdlib.h>
#include <math.h>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>
#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sys/time.h>


/**************Eigen头文件*******************/
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Cholesky>

/**************KDL库头文件*******************/
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>

using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;
using namespace std;
using namespace KDL;

/*********************************KDL运动学相关函数**************************************/
//左臂正解函数：已重写，改用Eigen数据类型
//输入VectorXd类型关节角向量、结构树
//输出VectorXd类型手掌坐标系的位姿x y z ox oy oz ow（注意：手掌在KDL中应该是第8连杆）
void forward_kinematics_left(VectorXd joint_values, Tree walker_tree, VectorXd &result);

//右臂正解函数：已重写，改用Eigen数据类型
//输入VectorXd类型关节角向量、结构树
//输出VectorXd类型手掌坐标系的位姿x y z ox oy oz ow（注意：手掌在KDL中应该是第8连杆）
void forward_kinematics_right(VectorXd joint_values, Tree walker_tree, VectorXd &result);

// 右臂目标抓取姿态赋值，根据Y方向位置分别划分为三个区域赋值
// 输入目标点位置，输出目标点姿态
VectorXd get_target_orientation_right(VectorXd target_position_right);

// 左臂目标抓取姿态赋值，根据Y方向位置分别划分为三个区域赋值
// 输入目标点位置，输出目标点姿态
VectorXd get_target_orientation_left(VectorXd target_position_left);

//底层反解函数：已重写，改用Eigen数据类型
//输入目标位置，目标姿态，反解初值种子，关节链，返回反解关节角
//注意：本函数不做超限判断，仅做求解，属于底层
VectorXd inverse_kinematics(VectorXd position, VectorXd orientation, VectorXd seed, KDL::Chain arm_chain);

//左右臂上层反解函数，调用上面反解函数
//输入URDF路径（用于提取关节链）、目标位置、目标姿态，输出反解关节角。返回是否成功反解，true表示成功，false表示失败
bool get_inverse_right(std::string urdf_path, VectorXd target_position_right, VectorXd target_orientation_right, VectorXd &ik_q_right);
bool get_inverse_left(std::string urdf_path, VectorXd target_position_left, VectorXd target_orientation_left, VectorXd &ik_q_left);


/********************************* 运动规划功能函数******************************************/
//手臂关节驱动函数
void robotPositionController(ros::Publisher &pub, VectorXd qd);


//三次多项式函数
double triple(double x0,double x1,double v0,double v1,double T,double t);

// Hank 20200115
// 2点带速度运动规划函数
VectorXd motionP2P_v2v(VectorXd q_init, VectorXd q_goal, VectorXd v_init, VectorXd v_goal, double duration,  double t);

// Hank 20200115
// 多点速度连续轨迹规划，起点、终点速度为0
// waypoints(N,7)是路径点，N*7矩阵, N表示第几个路径点，7表示7个关节
// t_是各段路径段时间，是N-1列向量(N个点只对应N-1段路径，只需要N-1个时间)
VectorXd multipoints_trajectory_plan(MatrixXd waypoints, VectorXd t_, double t);





#endif
