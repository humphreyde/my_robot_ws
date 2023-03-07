#ifndef ROBOT_KINEMATICS_SOLVER
#define ROBOT_KINEMATICS_SOLVER

#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <math.h>

using namespace std;
using namespace Eigen;

#define deg_to_rad M_PI/180
#define rad_to_deg 180/M_PI
#define randf ((rand()%10000+rand()%10000*10000)/100000000.0) //产生0-1随机浮点数
#define DATA_SIZE 500

const int NUM = 300;  //粒子总数
const float c1 = 1.65;  //速度计算参数1
const float c2 = 1.65;  //速度计算参数2
const float xmin = 0;   //位置下限
const float xmax = 20;    //位置上限


const float vmin = -1.5;   //速度下限
const float vmax = 1.5;    //速度上线

//robot结构常数，切勿修改
const double Dbs = 0.090;		//手臂基座长度
const double Dso = 0.247;		//大臂长度
const double Deo = 0;		//偏置距离
const double Dow = 0.211;		//小臂长度
const double Dwt = 0.2405;			//工具坐标系长度，根据需要调整，可以置0

struct particle
{
	float x[DATA_SIZE];   //该粒子的当前时刻位置
	float bestx[DATA_SIZE];//该粒子的历史最优位置
	float f;       //该粒子的当前适应度
	float bestf;   //该粒子的历史最优适应度
};   //总共有num个粒子


//连杆DH类，便于为杆件参数赋值
class Link
{
public:
	double theta;
	double d;
	double a;
	double alpha;

	Link(double theta_, double d_, double a_, double alpha_); //带参构造函数
	void printLinkInfo(); //返回连杆属性信息
};

//求解器类
class RobotKinematicsSolver
{

private:
	//连杆向量
	//注意：Eigen Vector3d默认是列向量
	Vector3d lbs_0, lse_3, leo_4, low_4, lwt_7;

	//左臂坐标系0到base_link的齐次变换矩阵
	Matrix4d T_left0_base;
	//base_link到左臂坐标系0的齐次变换矩阵
	Matrix4d T_base_left0;

	//不分左右，夹爪坐标系到坐标系7的齐次变换矩阵
	Matrix4d T_gripper_7;
	//不分左右，坐标系7到夹爪坐标系的齐次变换矩阵
	Matrix4d T_7_gripper;

	//基本正解函数：相对【坐标系0】，不直接使用，仅供左右臂正解函数调用
	//输入：不分左右臂，预期关节转角（从theta的初始offset处开始计算，代码自动将其转换为从坐标系零点开始的转角）
	//返回：不分左右臂，【夹爪】相对【坐标系0】的位姿齐次变换矩阵
	Matrix4d getEEPoseInC0(VectorXd q_expect);

	//基本解析反解函数：相对【坐标系0】，不直接使用，仅供左右臂反解函数调用
	//输入：不分左右臂，【夹爪】相对【坐标系0】的位置向量和姿态矩阵，臂形角
	//返回：不分左右臂，16组预期关节转角解（从theta的初始offset处开始计算的角度）
	vector<VectorXd> get16IkFromPoseInC0(Matrix4d Tgripper_0d, double psi);

	//ROBOT 基本解析反解函数：相对【坐标系0】，不直接使用，仅供左右臂反解函数调用
	//输入：不分左右臂，【夹爪】相对【base_link】的位置向量和姿态矩阵
	//返回：不分左右臂，8组预期关节转角解（从theta的初始offset处开始计算的角度）
	vector<VectorXd> get8IkInC0(Matrix4d T_gripper_0d,double q1);

	/*
	目标函数
	输入x[]为粒子位置
	dim，为每个粒子的位置参数个数，即一组解有多少个：7
	*/
	float f1(float x[], int dim);


public:
	//左右臂连杆变量，由于左右臂参数完全一致，定义一个两侧公用变量即可
	Link link1, link2, link3, link4, link5, link6, link7;

	//构造函数：初始化
	RobotKinematicsSolver();

	//求相邻连杆间的旋转矩阵
	//输入：theta——关节变量，即连杆实际旋转角度（包括初始角度在内）；alpha——关节扭角，属于固定结构参数，从DH参数表获取
	//返回：旋转矩阵
	Matrix3d calRotMatrix(double theta, double alpha);

	//求相邻连杆间的齐次变换矩阵
	//输入：theta——关节变量，即连杆实际旋转角度（包括初始角度在内）；d；a；alpha
	//返回：齐次变换矩阵
	Matrix4d calTransMatrix(double theta, double d, double a, double alpha);

	//求给定向量的反对称矩阵
	//输入：三维向量x
	//返回：3 * 3反对称矩阵u_hat
	Matrix3d calAntiMatrix(Vector3d x);

	//将按ZYX顺序，绕【运动坐标系轴】旋转的欧拉角转换为旋转矩阵
	//输入：三轴旋转量
	//返回：旋转矩阵
	Matrix3d calRotFromMovedZYX(double rz, double ry, double rx);

	//将按XYZ顺序，绕【固定坐标系轴】旋转的RPY角转换为旋转矩阵
	//输入：三轴旋转量
	//返回：旋转矩阵
	Matrix3d calRotFromFixedRPY(double rx, double ry, double rz);

	//左臂正解函数：相对base_link，输出齐次变换矩阵
	//输入：左臂预期关节转角（从theta的初始offset处开始计算，代码自动将其转换为从坐标系零点开始的转角）
	//返回：左臂【夹爪】相对base_link的位姿齐次变换矩阵
	Matrix4d getEETransMatInBase(VectorXd q_expect);

	//【常用】左臂正解函数：相对base_link，输出ROS geometry_msgs类型的位姿
	//输入：左臂预期关节转角（从theta的初始offset处开始计算，代码自动将其转换为从坐标系零点开始的转角）
	//返回：左臂【夹爪】相对base_link的geometry_msgs类型的位姿
	geometry_msgs::Pose getEEPoseInBase(VectorXd q_expect);


	//获取左臂所有坐标系相对base_link的geometry_msgs类型位姿，用于碰撞检测时碰撞体位姿计算
	//输入：左臂预期关节转角（从theta的初始offset处开始计算，代码自动将其转换为从坐标系零点开始的转角）
	//返回：左臂所有坐标系位姿组成的向量
	vector<geometry_msgs::Pose> getLeftAllPoseInBase(VectorXd q_expect);


	//【常用】高层——左臂臂形角迭代直至获取最优反解结果，geometry_msg类型位姿
	//输入：左臂【夹爪】相对【base_link】的geometry_msg类型位姿，存放反解结果的容器
	//输出：若未超限，输出左臂的一组最优反解关节角（从theta的初始offset处开始计算的角度）
	//返回：反解成功/失败
	bool getLeftBestIkFromPoseInBase(geometry_msgs::Pose left_pose, VectorXd &left_ik_value);


	//中层——左臂解析反解函数：相对【base_link】
	//输入：左臂【夹爪】相对【base_link】的位置向量和姿态矩阵，臂形角
	//返回：左臂16组预期关节转角解（从theta的初始offset处开始计算的角度）
	vector<VectorXd> getLeft16IkFromPoseInBase(Matrix4d T_leftGripper_base_d, double psi);


	//左臂关节角合理性验证函数
	//输入：左臂关节角
	//返回：是否超限
	bool robotJointCheck(VectorXd joint_value);



	//左臂解析反解函数：相对【base_link】
	//输入：左臂【夹爪】相对【base_link】的位置向量和姿态矩阵，臂形角
	//返回：左臂16组预期关节转角解（从theta的初始offset处开始计算的角度）
	vector<VectorXd> get8IkInBase(Matrix4d T_leftGripper_base_d, double theta1);
	

	//次高层——获取限制范围内可行反解结果，geometry_msg类型位姿
	//输入：左臂【夹爪】相对【base_link】的geometry_msg类型位姿，存放反解结果的容器
	//输出：若未超限，输出左臂的一组最优反解关节角（从theta的初始offset处开始计算的角度）
	//返回：反解成功/失败
	vector<VectorXd> getLimitIkInBase(Matrix4d T_leftGripper_base_d,double theta1);

	//高层——左臂臂形角迭代直至获取最优反解结果，geometry_msg类型位姿
	//输入：左臂【夹爪】相对【base_link】的geometry_msg类型位姿，存放反解结果的容器
	//输出：若未超限，输出左臂的一组最优反解关节角（从theta的初始offset处开始计算的角度）
	//返回：反解成功/失败
	bool getBestIkFromPoseInBase(geometry_msgs::Pose left_gripper_pose, VectorXd &left_ik_value);

	/*
	best为全局最优位置
	DIM为目标函数的输入参数总个数，也即每个粒子的位置参数总个数：7,  每个粒子是一组解   //NUM个粒子，DATA_SIZE个位置
	*/
	void PSO(float* best, int DIM);

};

#endif
