#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <stdio.h>
#include <iterator>
#include <algorithm>
#include <sys/time.h>
#include "robot_kinematics_solver/robot_kinematics_solver.h"

using namespace std;
using namespace Eigen;

//连杆类构造函数
Link::Link(double theta_, double d_, double a_, double alpha_)
{
	theta = theta_;
	d = d_;
	a = a_;
	alpha = alpha_;
}

//输出连杆参数
void Link::printLinkInfo()
{
	cout << "theta: " << theta << "\t d: " << d << "\t a: " << a << "\t alpha: " << alpha << endl;
}


//求解器类构造函数：公用变量初始化
RobotKinematicsSolver::RobotKinematicsSolver() //初始化连杆DH参数表
	: link1(0,  	-Dbs,   	0,     -M_PI / 2),
	  link2(0,  	0,     			0,     -M_PI / 2),
	  link3(0,      Dso,  		0,  	M_PI / 2),
	  link4(0,       0,     		0,   -M_PI / 2),
	  link5(0,   	Dow,   		0,    M_PI / 2),
	  link6(0,   	0,     			0,    -M_PI / 2),
	  link7(0,     	Dwt,		0,    	 0       )
{
	//初始化连杆向量
	//注意：Eigen Vector3d默认是列向量
	lbs_0 <<  0,    0,   -Dbs;
	lse_3 << -Deo, -Dso,  0;
	leo_4 <<  Deo,  0,    0;
	low_4 <<  0,    0,    Dow;
	lwt_7 <<  0,  0,    Dwt;
	
	//计算【左臂】坐标系0相对base_link的变换
	//旋转变换
	Matrix3d R_left0_base;
	R_left0_base = calRotFromFixedRPY(-M_PI/2, 0, 0); //由robot结构和环境得出,其base和世界坐标系重合
	//平移变换
	Vector3d t_left0_base;
	t_left0_base << 0, 1.435, 0;//由robot处的环境得出，坐标系0是关节1的旋转基准，坐标系1是关节2的旋转基准，是link1的坐标系，坐标系7是夹爪坐标系
	//综合为齐次变换矩阵
	T_left0_base << R_left0_base, t_left0_base, 0, 0, 0, 1;

	//求base_link到左臂坐标系0的变换，也就是T_left0_base的逆变换
	T_base_left0 = T_left0_base.inverse();

	//计算【夹爪】到【坐标系7——球腕】的变换，注意这一变换不分左右臂
	//平移变换
	Vector3d t_gripper_7;
	t_gripper_7 << 0, 0, 0;//目前为dwt
    //旋转变换：绕C7 Z轴旋转45度
	Matrix3d R_gripper_7;
	R_gripper_7 = calRotFromFixedRPY(0, 0, 0);//目前没有
	//综合为齐次变换矩阵
	T_gripper_7 << R_gripper_7, t_gripper_7, 0, 0, 0, 1;

	//求计算【坐标系7——球腕】到【夹爪】的变换，也就是上一变换的逆变换，注意这一变换不分左右臂
	T_7_gripper = T_gripper_7.inverse();

}

//求相邻连杆间的旋转矩阵
//输入：theta——关节变量，即连杆实际旋转角度（包括初始角度在内）；alpha——关节扭角，属于固定结构参数，从DH参数表获取
//返回：旋转矩阵
Matrix3d RobotKinematicsSolver::calRotMatrix(double theta, double alpha)
{
	Matrix3d R;
	R << cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha),
		 sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha),
		 0,           sin(alpha),             cos(alpha);
	return R;
}

//求相邻连杆间的齐次变换矩阵
//输入：theta——关节变量，即连杆实际旋转角度（包括初始角度在内）；d；a；alpha
//返回：齐次变换矩阵
Matrix4d RobotKinematicsSolver::calTransMatrix(double theta, double d, double a, double alpha)
{
	Matrix4d T;
	T << cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta),
		 sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
		 0,           sin(alpha),             cos(alpha),            d,
		 0,           0,                      0,                     1;
	return T;
}

//求给定向量的反对称矩阵
//输入：三维向量x
//返回：3 * 3反对称矩阵u_hat
Matrix3d RobotKinematicsSolver::calAntiMatrix(Vector3d x)
{
	Vector3d u = x / x.norm(); //求x向量的单位向量
	Matrix3d u_hat;
	//注意：Eigen的序号从0开始！
	u_hat << 0,    -u(2),  u(1),
		     u(2),  0,    -u(0),
		    -u(1),  u(0),  0;

	return u_hat;
}

//将按ZYX顺序，绕【运动坐标系轴】旋转的欧拉角转换为旋转矩阵
//输入：三轴旋转量
//返回：旋转矩阵
Matrix3d RobotKinematicsSolver::calRotFromMovedZYX(double rz, double ry, double rx)
{
	//预先计算正余弦值
	double cz = cos(rz); double sz = sin(rz);
	double cy = cos(ry); double sy = sin(ry);
	double cx = cos(rx); double sx = sin(rx);

	//求等价旋转矩阵
	Matrix3d Rz, Ry, Rx, R;
	Rz << cz, -sz, 0,
		  sz,  cz, 0,
		  0,   0,  1;

	Ry << cy, 0, sy,
		  0,  1, 0,
		 -sy, 0, cy;

	Rx << 1, 0,   0, 
		  0, cx, -sx, 
	      0, sx,  cx;

	R = Rz*Ry*Rx;

	return R;
}

//将按XYZ顺序，绕【固定坐标系轴】旋转的RPY角转换为旋转矩阵（表达式与上个函数相同）
//输入：三轴旋转量
//返回：旋转矩阵
Matrix3d RobotKinematicsSolver::calRotFromFixedRPY(double rx, double ry, double rz)
{
	//预先计算正余弦值
	double cz = cos(rz); double sz = sin(rz);
	double cy = cos(ry); double sy = sin(ry);
	double cx = cos(rx); double sx = sin(rx);

	//求等价旋转矩阵
	Matrix3d Rz, Ry, Rx, R;
	Rz << cz, -sz, 0,
		sz, cz, 0,
		0, 0, 1;

	Ry << cy, 0, sy,
		0, 1, 0,
		-sy, 0, cy;

	Rx << 1, 0, 0,
		0, cx, -sx,
		0, sx, cx;

	R = Rz*Ry*Rx;

	return R;
}

//基本正解函数：相对【坐标系0】，不直接使用，仅供左右臂正解函数调用
//输入：不分左右臂，预期关节转角（从theta的初始offset处开始计算，代码自动将其转换为从坐标系零点开始的转角）
//返回：不分左右臂，相对【坐标系0】的位姿齐次变换矩阵
Matrix4d RobotKinematicsSolver::getEEPoseInC0(VectorXd q_expect)
{
	// //连杆间旋转矩阵变量
	// Matrix3d R1_0, R2_1, R3_2, R4_3, R5_4, R6_5, R7_6;
	// Matrix3d R7_4, R3_0;

	// //本正解算法是从最原始的、定义的关节坐标系零点开始计算的
	// //所以给预期关节角要另加关节2、6处的原始offset
	VectorXd q_init_offset(7), q_from_zero(7);
	//q_init_offset << -M_PI / 2, M_PI / 2, M_PI, 0, M_PI / 2, - M_PI / 2, 0;
	q_init_offset << 0, 0, 0, 0, 0, 0, 0;//offset 为0

	//计算从原点开始的关节转角
	q_from_zero = q_expect + q_init_offset;

	// //求相邻旋转矩阵
	// //注意：C++与MATLAB不同，计数序号从0开始
	// R1_0 = calRotMatrix(q_from_zero(0), link1.alpha);
	// R2_1 = calRotMatrix(q_from_zero(1), link2.alpha);
	// R3_2 = calRotMatrix(q_from_zero(2), link3.alpha);
	// R4_3 = calRotMatrix(q_from_zero(3), link4.alpha);
	// R5_4 = calRotMatrix(q_from_zero(4), link5.alpha);
	// R6_5 = calRotMatrix(q_from_zero(5), link6.alpha);
	// R7_6 = calRotMatrix(q_from_zero(6), link7.alpha);

	// //连乘求2个重要不相邻关节旋转矩阵
	// R7_4 = R5_4 * R6_5 * R7_6;
	// R3_0 = R1_0 * R2_1 * R3_2;

	// //计算相对坐标系0的末端正解位姿
	// Vector3d x7_0;
	// x7_0 = lbs_0 + R3_0 * (lse_3 + R4_3 * (leo_4 + low_4 + R7_4 * lwt_7)); //位置
	// Matrix3d R7_0;
	// R7_0 = R3_0 * R4_3 * R7_4; //姿态

	//综合为7到0的齐次坐标
	Matrix4d T1_0,  T2_1, T3_2, T4_3, T5_4, T6_5, T7_6, T7_0;

	T1_0 = calTransMatrix(q_from_zero(0), link1.d, link1.a, link1.alpha);
	T2_1 = calTransMatrix(q_from_zero(1), link2.d, link2.a, link2.alpha);
	T3_2 = calTransMatrix(q_from_zero(2), link3.d, link3.a, link3.alpha);
	T4_3 = calTransMatrix(q_from_zero(3), link4.d, link4.a, link4.alpha);
	T5_4 = calTransMatrix(q_from_zero(4), link5.d, link5.a, link5.alpha);
	T6_5 = calTransMatrix(q_from_zero(5), link6.d, link6.a, link6.alpha);
	T7_6 = calTransMatrix(q_from_zero(6), link7.d, link7.a, link7.alpha);
	T7_0  = T1_0 * T2_1 * T3_2 * T4_3 * T5_4 * T6_5 * T7_6; //矩阵连接

	//转换为夹爪相对于坐标系0的位姿
	Matrix4d Tgripper_0 = T7_0 * T_gripper_7;

	return Tgripper_0;
}

//左臂正解函数：相对base_link，输出齐次变换矩阵
//输入：左臂预期关节转角（从theta的初始offset处开始计算，代码自动将其转换为从坐标系零点开始的转角）
//返回：左臂末端相对base_link的位姿齐次变换矩阵
Matrix4d RobotKinematicsSolver::getEETransMatInBase(VectorXd q_expect)
{
	//调用【基本正解函数】获取左臂末端在左臂坐标系0中的位姿齐次变换矩阵
	Matrix4d T_leftGripper_left0;
	T_leftGripper_left0 = getEEPoseInC0(q_expect);

	//返回变换后的末端位姿
	Matrix4d T_leftGripper_base;
	T_leftGripper_base = T_left0_base * T_leftGripper_left0;

	return T_leftGripper_base;
}

//左臂正解函数：相对base_link，输出ROS geometry_msgs类型的位姿
//输入：左臂预期关节转角（从theta的初始offset处开始计算，代码自动将其转换为从坐标系零点开始的转角）
//返回：左臂末端相对base_link的geometry_msgs类型的位姿
geometry_msgs::Pose RobotKinematicsSolver::getEEPoseInBase(VectorXd q_expect)
{
	Matrix4d T = getEETransMatInBase(q_expect);
	//将齐次变换矩阵转换为ros类型
	geometry_msgs::Pose pose;
	//位置可以直接赋值
	pose.position.x = T(0, 3); //取1行4列
	pose.position.y = T(1, 3); //取1行4列
	pose.position.z = T(2, 3); //取1行4列
	//姿态需要经过旋转矩阵转化为四元数
	Matrix3d R = T.block<3, 3>(0, 0); //取小块，从1行1列开始的三行三列
	//利用Eigen将R转换为四元数
	Quaterniond q(R); //用旋转矩阵初始化四元数，等价于转换
	//姿态赋值
	pose.orientation.x = q.x(); //务必注意Eigen四元数的储存顺序
	pose.orientation.y = q.y();
	pose.orientation.z = q.z();
	pose.orientation.w = q.w();

	return pose;
}

//获取左臂所有坐标系相对base_link的geometry_msgs类型位姿，用于碰撞检测时碰撞体位姿计算
//输入：左臂预期关节转角（从theta的初始offset处开始计算，代码自动将其转换为从坐标系零点开始的转角）
//返回：左臂所有坐标系位姿组成的向量
vector<geometry_msgs::Pose> RobotKinematicsSolver::getLeftAllPoseInBase(VectorXd q_expect)
{
	//本正解算法是从最原始的、定义的关节坐标系零点开始计算的
	//所以给预期关节角要另加关节2、6处的原始offset
	VectorXd q_init_offset(7), q_from_zero(7);
	q_init_offset << -M_PI / 2, M_PI / 2, M_PI, 0, M_PI / 2, - M_PI / 2, 0;

	//计算从原点开始的关节转角
	q_from_zero = q_expect + q_init_offset;

	//给定关节角，计算各个连杆的齐次变换矩阵
	Matrix4d T1_0, T2_1, T3_2, T4_3, T5_4, T6_5, T7_6;
	T1_0 = calTransMatrix(q_from_zero(0), link1.d, link1.a, link1.alpha);
	T2_1 = calTransMatrix(q_from_zero(1), link2.d, link2.a, link2.alpha);
	T3_2 = calTransMatrix(q_from_zero(2), link3.d, link3.a, link3.alpha);
	T4_3 = calTransMatrix(q_from_zero(3), link4.d, link4.a, link4.alpha);
	T5_4 = calTransMatrix(q_from_zero(4), link5.d, link5.a, link5.alpha);
	T6_5 = calTransMatrix(q_from_zero(5), link6.d, link6.a, link6.alpha);
	T7_6 = calTransMatrix(q_from_zero(6), link7.d, link7.a, link7.alpha);

	//计算各连杆相对坐标系0的齐次变换矩阵
	Matrix4d T2_0, T3_0, T4_0, T5_0, T6_0, T7_0;
	T2_0 = T1_0 * T2_1;
	T3_0 = T2_0 * T3_2;
	T4_0 = T3_0 * T4_3;
	T5_0 = T4_0 * T5_4;
	T6_0 = T5_0 * T6_5;
	T7_0 = T6_0 * T7_6;

	vector<Matrix4d> all_transmats;
	all_transmats.push_back(T1_0);
	all_transmats.push_back(T2_0);
	all_transmats.push_back(T3_0);
	all_transmats.push_back(T4_0);
	all_transmats.push_back(T5_0);
	all_transmats.push_back(T6_0);
	all_transmats.push_back(T7_0);

	//此外加入末端抓手的位姿
	Matrix4d Tgripper_0;
	Tgripper_0 = T7_0 * T_gripper_7;
	all_transmats.push_back(Tgripper_0);
	
	vector<geometry_msgs::Pose> all_poses;
	for (int i=0; i<all_transmats.size(); ++i)
	{
		//转换为相对base_link的齐次矩阵
		Matrix4d Tx_base;
		Tx_base = T_left0_base * all_transmats[i];

		//转换为geometry_msgs::Pose类型位姿
		geometry_msgs::Pose pose;
		//位置可以直接赋值
		pose.position.x = Tx_base(0, 3); //取1行4列
		pose.position.y = Tx_base(1, 3); //取1行4列
		pose.position.z = Tx_base(2, 3); //取1行4列
		//姿态需要经过旋转矩阵转化为四元数
		Matrix3d R = Tx_base.block<3, 3>(0, 0); //取小块，从1行1列开始的三行三列
		//利用Eigen将R转换为四元数
		Quaterniond q(R); //用旋转矩阵初始化四元数，等价于转换
		//姿态赋值
		pose.orientation.x = q.x(); //务必注意Eigen四元数的储存顺序
		pose.orientation.y = q.y();
		pose.orientation.z = q.z();
		pose.orientation.w = q.w();

		all_poses.push_back(pose);
	}
	
	return all_poses;
}



//基本解析反解函数：相对【坐标系0】，不直接使用，仅供左右臂反解函数调用（臂形角法）
//输入：不分左右臂，【夹爪】相对【坐标系0】的位置向量和姿态矩阵，臂形角
//返回：不分左右臂，16组预期关节转角解（从theta的初始offset处开始计算的角度）
vector<VectorXd> RobotKinematicsSolver::get16IkFromPoseInC0(Matrix4d Tgripper_0d, double psi)
{
	//将预期夹爪位姿转换为预期坐标系7的位姿
	Matrix4d T7_0d;
	T7_0d = Tgripper_0d * T_7_gripper;

	//齐次矩阵分解为平移向量和旋转矩阵
	Vector3d t7_0d;
	Matrix3d R7_0d;

	//分解为位置和姿态
	t7_0d = T7_0d.block<3, 1>(0, 3); //取小块
	R7_0d = T7_0d.block<3, 3>(0, 0); //取小块

	double SE = sqrt(pow(Dso, 2) + pow(Deo, 2)); //推理详情见文档
	double EW = sqrt(pow(Deo, 2) + pow(Dow, 2)); 
	double SW0 = Dso + Dow;

	//SW向量的坐标表示
	Vector3d Xsw_0 = t7_0d - lbs_0 - R7_0d * lwt_7;
	double SW = Xsw_0.norm(); //任意情况下Xsw_0的模长

	//求解theta40、theta4t
	double theta40 = acos((pow(SE, 2) + pow(EW, 2) - pow(SW0, 2)) / (2 * SE * EW));
	double theta4t = acos((pow(SE, 2) + pow(EW, 2) - pow(SW, 2)) / (2 * SE * EW));

	//求解theta4，实际只有2个解
	//但为了与后续R300解数对应，取为重复的4解，存入vector中便于迭代
	vector<double> theta4(4);
	theta4[0] = -(2 * M_PI - (theta40 + theta4t)); //反转解（robot实际只能反转）
	theta4[1] = theta4[0];						 //重复反转解（robot实际只能反转）
	theta4[2] = theta40 - theta4t;               //正转解
	theta4[3] = theta4[2];                       //重复正转解

	//求y300
	//计算公用值：alpha和beta
	double alpha = acos((pow(Dso, 2) + pow(SW, 2) - pow(Dow, 2)) / (2 * sqrt(pow(Dso, 2) + pow(Deo, 2)) * SW));
	double beta = atan2(Deo, Dso);

	//计算公用值：单位l向量及其反对称矩阵
	Vector3d l = lbs_0.cross(Xsw_0);	  //l向量
	Matrix3d Ul_hat = calAntiMatrix(l);   //计算l向量的单位反对称矩阵
	Matrix3d I3 = Matrix3d::Identity();   //定义一个单位矩阵
	//一些公用计算矩阵变量，避免多次计算
	Matrix3d Ul_hat_square = Ul_hat * Ul_hat;
	Vector3d Usw_0 = Xsw_0 / SW; //单位SW向量
	Matrix3d Usw_0_hat = calAntiMatrix(Xsw_0); //单位SW向量的反对称矩阵
	
	//根据罗德里格斯公式求y300的4组解
	//根据正反转以及z、l正反向状态的不同，旋转角Gamma共4种情况，存入向量
	vector<double> gamma(4);
	gamma[0] = alpha + beta;   //1. z300和l同向 + 肘关节反转情况：a + b
	gamma[1] = -alpha - beta;  //2. z300和l反向 + 肘关节反转情况：-(a + b)
	gamma[2] = alpha - beta;   //3. z300和l反向 + 肘关节正转情况：a - b
	gamma[3] = -alpha + beta;  //4. z300和l同向 + 肘关节正转情况：b - a

	//计算R_l临时旋转矩阵和y300向量对应4解都，放入vector，便于迭代
	vector<Matrix3d> R_l(4);
	vector<Vector3d> y300(4);
	for(int i=0; i<4; ++i)
	{
		R_l[i] = I3 + Ul_hat * sin(gamma[i]) + Ul_hat_square * (1 - cos(gamma[i]));
		y300[i] = -R_l[i] * Usw_0;
	}

	//计算4组z300，解序号与y300的取值保持一致
	vector<Vector3d> z300(4);
	z300[0] = l / l.norm();
	z300[1] = -z300[0];
	z300[2] = -z300[0];
	z300[3] = z300[0];

	//计算x300，解序号与另两者同样保持一致
	vector<Vector3d> x300(4);
	for(int i=0; i<4; ++i)
	{
		x300[i] = y300[i].cross(z300[i]);
	}

	//综合为R300，一共4解
	//将R300四解存入vector中便于后续迭代求解
	vector<Matrix3d> R300(4);
	for(int i=0; i<4; ++i)
	{
		R300[i] << x300[i], y300[i], z300[i];
	}

	//预先计算臂形角psi的正余弦值
	double cpsi = cos(psi);
	double spsi = sin(psi);

	//求解肩关节、腕关节，综合最终解
	//最终16组解的容器
	vector<VectorXd> ik_16_values; //不要初始化大小，否则不能使用push_back
	for (int i = 0; i < 4; ++i)
	{
		Matrix3d As = Usw_0_hat * R300[i];
		Matrix3d Bs = -Usw_0_hat * Usw_0_hat * R300[i];
		Matrix3d Cs = (Usw_0 * Usw_0.transpose()) * R300[i];
		
		//求给定臂形角下的R30
		Matrix3d R30_psi = As * spsi + Bs * cpsi + Cs;

		//取R30_psi矩阵特殊位上的元素
		//注意：Eigen元素序号从0开始！
		double M31 = R30_psi(2, 0);
		double M32 = R30_psi(2, 1);
		double M33 = R30_psi(2, 2);
		double M12 = R30_psi(0, 1);
		double M22 = R30_psi(1, 1);

		//求theta2、1、3的2组解
		vector<double> theta1(2), theta2(2), theta3(2);
		theta2[0] = acos(M32); //theta2有正负2值
		theta2[1] = -theta2[0];
		for (int j = 0; j < 2; ++j)
		{
			theta1[j] = atan2(-M22 * sin(theta2[j]), -M12 * sin(theta2[j]));
			theta3[j] = atan2(-M33 * sin(theta2[j]), M31 * sin(theta2[j]));
		}

		Matrix3d R4_3_psi = calRotMatrix(theta4[i], link4.alpha);
		Matrix3d R4_3_psi_transpose = R4_3_psi.transpose(); //避免重复计算
		Matrix3d Aw = R4_3_psi_transpose * As.transpose() * R7_0d;
		Matrix3d Bw = R4_3_psi_transpose * Bs.transpose() * R7_0d;
		Matrix3d Cw = R4_3_psi_transpose * Cs.transpose() * R7_0d;

		//求给定臂形角下的R74
		Matrix3d R74_psi = Aw * spsi + Bw * cpsi + Cw;

		//取R74_psi矩阵特殊位上的元素
		//注意：Eigen元素序号从0开始！
		double W13 = R74_psi(0, 2);
		double W23 = R74_psi(1, 2);
		double W31 = R74_psi(2, 0);
		double W32 = R74_psi(2, 1);
		double W33 = R74_psi(2, 2);

		//求theta6、5、7的2组解
		vector<double> theta5(2), theta6(2), theta7(2);
		theta6[0] = acos(W33); //theta6有正负2值
		theta6[1] = -theta6[0];
		for (int j = 0; j < 2; ++j)
		{
			theta5[j] = atan2(W23 * sin(theta6[j]), W13 * sin(theta6[j]));
			theta7[j] = atan2(W32 * sin(theta6[j]), -W31 * sin(theta6[j]));
		}

		//综合所有解
		VectorXd joint_value_temp(7); 
		for (int j = 0; j < 2; ++j) //关于theta2系列的迭代
		{
			for (int k = 0; k < 2; k++) //关于theta6系列的迭代
			{   //                   各计算关节角减去关节初始Offset才是最终预期转角
				joint_value_temp(0) = theta1[j] - (-M_PI / 2);
				joint_value_temp(1) = theta2[j] - (M_PI / 2);
				joint_value_temp(2) = theta3[j] - M_PI;
				joint_value_temp(3) = theta4[i];
				joint_value_temp(4) = theta5[k] - (M_PI / 2);
				joint_value_temp(5) = theta6[k] - (-M_PI / 2);
				joint_value_temp(6) = theta7[k];
				ik_16_values.push_back(joint_value_temp);
			}
		}
	}

	return ik_16_values;
}

//基本解析反解函数：相对【坐标系0】，不直接使用，仅供左右臂反解函数调用（关节角参数）
//输入：【夹爪】相对【坐标系0】的位置向量和姿态矩阵，关节角参数
//返回：8组预期关节转角解（从theta的初始offset处开始计算的角度）
vector<VectorXd> RobotKinematicsSolver::get8IkInC0(Matrix4d T_gripper_0d,double theta1)
{
	//theta4两种取值，theta3、theta2两种，theta6两种，theta5，theta7一种。
	//将预期夹爪位姿转换为预期坐标系7的位姿
	Matrix4d T7_0d;
	T7_0d = T_gripper_0d * T_7_gripper;
	//齐次矩阵分解为平移向量和旋转矩阵
	Vector3d t7_0d;
	Matrix3d R7_0d;

	//分解为位置和姿态
	t7_0d = T7_0d.block<3, 1>(0, 3); //取小块
	R7_0d = T7_0d.block<3, 3>(0, 0); //取小块

	double SE = Dso; //推理详情见文档
	double EW = Dow; 
	double SW0 = Dso + Dow;

	//SW向量的坐标表示
	Vector3d Xsw_0 = t7_0d - lbs_0 - R7_0d * lwt_7;
	double SW = Xsw_0.norm(); //任意情况下Xsw_0的模长


	double theta40 = acos((pow(SE, 2) + pow(EW, 2) - pow(SW, 2)) / (2 * SE * EW));

	//求解theta4，实际只有2个解
	//但为了与后续R300解数对应，取为重复的4解，存入vector中便于迭代
	vector<double> theta4(4);
	
	theta4[0] =  M_PI - theta40 ; 				//反转解（robot实际只能反转）
	theta4[1] = theta4[0];						 //重复反转解（robot实际只能反转）
	theta4[2] = M_PI + theta40;               //正转解
	theta4[3] = theta4[2];                       //重复正转解
//	cout <<"两组theta4:"<< theta4[0] << theta4[2] << endl;
	//theta3
	vector<double> theta3(4);

	double q2, q3, q4, q5, q6, q7;
    

	Matrix4d T0_7 = T7_0d.inverse();

    Vector3d Tz = -R7_0d * (T0_7.block<3, 1>(0, 3) + lwt_7);
	
   // q3 = 3.141592- asin((double(Tz(0, 0) * sin(theta1) - Tz(1, 0) * cos(theta1))) / (-0.211 * sin(q4)));
	theta3[0] = asin((double(Tz(0, 0) * sin(theta1) - Tz(1, 0) * cos(theta1))) / -Dow* sin(theta4[0]));
	theta3[1] = M_PI - theta3[0];
	theta3[2] = asin((double(Tz(0, 0) * sin(theta1) - Tz(1, 0) * cos(theta1))) / -Dow* sin(theta4[2]));
	theta3[3] = M_PI - theta3[2];

	//theta2
	
	vector<double> theta2(4);
	double  A2, B2, C2, D2;
	C2 = double(Tz(0, 0) * cos(theta1) + Tz(1, 0) * sin(theta1));
	D2 = Tz(2, 0) + Dbs;
	for(int i = 0; i<4; i++)
	{
		A2 = Dso + Dow * cos(theta4[i]);
		B2 = Dow * sin(theta4[i]) * cos(theta3[i]);

		theta2[i] = atan((A2 * C2 - B2 * D2) / (B2 * C2 + A2 * D2));
	}
	
	//theta6  theta7 theta5
	vector<double> theta6(8);
	vector<double> theta7(8),theta5(8);
    Matrix4d T1,T2,T3,T4;

    T1 << cos(theta1), 0, -sin(theta1), 0, sin(theta1), 0, cos(theta1), 0, 0, -1, 0, -Dbs, 0, 0, 0, 1;
	for(int i = 0; i<4; i++)
	{
		T2 << cos(theta2[i]), 0, -sin(theta2[i]), 0, sin(theta2[i]), 0, cos(theta2[i]), 0, 0, -1, 0, 0, 0, 0, 0, 1;
		T3 << cos(theta3[i]), 0, sin(theta3[i]), 0, sin(theta3[i]), 0, -cos(theta3[i]), 0, 0, 1, 0, Dso, 0, 0, 0, 1;
		T4 << cos(theta4[i]), 0, -sin(theta4[i]), 0, sin(theta4[i]), 0, cos(theta4[i]), 0, 0, -1, 0, 0, 0, 0, 0, 1;
		Matrix4d T04 = T1 * T2 * T3 * T4;
		Matrix4d T47 = T04.inverse() * T7_0d;
		
		//theta6 
		theta6[i] = acos(T47(2, 2));
		theta6[i+4] = -acos(T47(2, 2));

		//theta7&theta5
		//theta7[i] = atan2(-T47(2, 1) / sin(theta6[i]), T47(2, 0) / sin(theta6[i]));
		//当sin（q6）无限小 
		theta7[i] = atan2(-T47(2, 1)*sin(theta6[i]), T47(2, 0)*sin(theta6[i]));
		theta7[i+4] = atan2(-T47(2, 1)*sin(theta6[i+4]), T47(2, 0)*sin(theta6[i+4]));
		//theta5[i] = -atan2(-T47(1, 2) / sin(theta6[i]), -T47(0, 2) / sin(theta6[i]));
		//当sin（q6）无限小 
		theta5[i] = atan2(-T47(1, 2) * sin(theta6[i]), -T47(0, 2) * sin(theta6[i]));
		theta5[i+4] = atan2(-T47(1, 2) * sin(theta6[i+4]), -T47(0, 2) * sin(theta6[i+4]));
	
	}
    

	//8组解
	vector<VectorXd> ik_8_values; //不要初始化大小，否则不能使用push_back
    
	VectorXd q,q1;
	q.resize(7);
	q1.resize(7);
	for(int i = 0; i<4; i++)
	{
		q << theta1, theta2[i], theta3[i], theta4[i], theta5[i],theta6[i],theta7[i];
		q1 << theta1, theta2[i], theta3[i], theta4[i], theta5[i+4],theta6[i+4],theta7[i+4];
		ik_8_values.push_back(q);
		ik_8_values.push_back(q1);
	}
	
	return ik_8_values;
}

//左臂解析反解函数：相对【base_link】
//输入：左臂【夹爪】相对【base_link】的位置向量和姿态矩阵，臂形角
//返回：左臂16组预期关节转角解（从theta的初始offset处开始计算的角度）
vector<VectorXd> RobotKinematicsSolver::getLeft16IkFromPoseInBase(Matrix4d T_leftGripper_base_d, double psi)
{
	//将输入的夹爪相对base的位姿转换为相对坐标系0的位姿
	Matrix4d T_leftGripper_left0_d;
	T_leftGripper_left0_d = T_base_left0 * T_leftGripper_base_d;

	//调用基本反解函数，获得16组解
	vector<VectorXd> left_ik_16_values;
	left_ik_16_values = get16IkFromPoseInC0(T_leftGripper_left0_d, psi);

	return left_ik_16_values;
}

//左臂解析反解函数：相对【base_link】
//输入：左臂【夹爪】相对【base_link】的位置向量和姿态矩阵，臂形角
//返回：左臂16组预期关节转角解（从theta的初始offset处开始计算的角度）
vector<VectorXd> RobotKinematicsSolver::get8IkInBase(Matrix4d T_leftGripper_base_d, double theta1)
{
	//将输入的夹爪相对base的位姿转换为相对坐标系0的位姿
	Matrix4d T_leftGripper_left0_d;
	T_leftGripper_left0_d = T_base_left0 * T_leftGripper_base_d;

	//调用基本反解函数，获得16组解
	vector<VectorXd> left_ik_8_values;
	left_ik_8_values = get8IkInC0(T_leftGripper_left0_d, theta1);

	return left_ik_8_values;
}


//左臂关节角合理性验证函数
//输入：左臂关节角
//返回：是否超限
bool RobotKinematicsSolver::robotJointCheck(VectorXd joint_value)
{
	bool is_available = false;
	if (joint_value(0) > -2.0944 && joint_value(0) < 2.0944 &&
		joint_value(1) > -1.5708 && joint_value(1) < 1.5708 &&
		joint_value(2) > -2.0944 && joint_value(2) < 2.0944 &&
		joint_value(3) > -1.5708 && joint_value(3) < 1.5708 &&
		joint_value(4) > -2.0944 && joint_value(4) < 2.0944 &&
		joint_value(5) > -1.5708 && joint_value(5) < 1.5708 &&
		joint_value(6) > -3.1416 && joint_value(6) < 3.1416)
	{
		is_available = true;
	}
	return is_available;
}


//高层——左臂臂形角迭代直至获取最优反解结果，geometry_msg类型位姿
//输入：左臂【夹爪】相对【base_link】的geometry_msg类型位姿，存放反解结果的容器
//输出：若未超限，输出左臂的一组最优反解关节角（从theta的初始offset处开始计算的角度）
//返回：反解成功/失败
bool RobotKinematicsSolver::getLeftBestIkFromPoseInBase(geometry_msgs::Pose left_gripper_pose, VectorXd &left_ik_value)
{
	//把输入的geometry_msgs::Pose类型的位姿转换为位置向量和旋转矩阵
	Vector3d t_leftGripper_base_d;
	t_leftGripper_base_d << left_gripper_pose.position.x, left_gripper_pose.position.y, left_gripper_pose.position.z;

	//利用Eigen把四元数转旋转矩阵
	//注意Eigen四元数的赋值顺序是WXYZ，输出顺序是XYZW
	Quaterniond q(left_gripper_pose.orientation.w, left_gripper_pose.orientation.x, left_gripper_pose.orientation.y,left_gripper_pose.orientation.z);
	Matrix3d R_leftGripper_base_d;
	R_leftGripper_base_d = q.toRotationMatrix();

	//组合为抓手相对base的齐次变换矩阵
	Matrix4d T_leftGripper_base_d;
	T_leftGripper_base_d << R_leftGripper_base_d, t_leftGripper_base_d, 0, 0, 0, 1;

	//求最优臂形角(暂无)

	//验证最优臂形角下是否有可行解(暂无)

	//若无可行解，开始循环迭代直至找到解
	double psi = -M_PI;

	vector<VectorXd> left_ik_available; //存放所有可用解的容器

	for (int j = 0; ; ++j)
	{
		if (psi > M_PI)
		{
			break;
		}

		//调用获取中层反解函数，获得全部16组解
		vector<VectorXd> left_ik_16_values;
		left_ik_16_values = getLeft16IkFromPoseInBase(T_leftGripper_base_d, psi);

		//获取未超限的解，放入容器
		for (int i = 0; i < left_ik_16_values.size(); ++i)
		{
			if (robotJointCheck(left_ik_16_values[i])) //16个解送入超限检查函数，没超限的话记录序号
			{
				left_ik_available.push_back(left_ik_16_values[i]);
			}
		}

		psi = psi + 0.2;
	}

	// cout << "迭代可行解数：" << left_ik_available.size()<< endl;
	if(left_ik_available.empty())
	{
		// cout << "未找到解！" << endl;
		return false;  //return后后续代码不会执行
	}
		
	//在可行解中找出距离各关节中心点最近的解，作为最优解输出
	//计算左臂关节转动限制中线值
	VectorXd left_arm_center(7);
	left_arm_center << 1.1781, -0.77665, 0, -1.117, 0, 0, 0; //中线值

	//计算所有可行解与中线值的差值向量的模，存入向量
	vector<double> diff_norm;
	for(int i=0; i<left_ik_available.size(); ++i)
	{
		diff_norm.push_back((left_ik_available[i]-left_arm_center).norm()); //求差值向量的模，存入vector便于找到最小值索引
		//cout << diff_norm[i] << endl; 
	}

	//找出最小值及其索引
	vector<double>::iterator smallest = min_element(diff_norm.begin(), diff_norm.end());
	int smallest_index = distance(diff_norm.begin(), smallest);

	//将最小索引对应的关节角赋值输出
    left_ik_value = left_ik_available[smallest_index];

	return true;
}



//次高层——获取限制范围内可行反解结果，geometry_msg类型位姿
//输入：左臂【夹爪】相对【base_link】的geometry_msg类型位姿，存放反解结果的容器
//输出：若未超限，输出左臂的一组最优反解关节角（从theta的初始offset处开始计算的角度）
//返回：反解成功/失败
vector<VectorXd> RobotKinematicsSolver::getLimitIkInBase(Matrix4d T_leftGripper_base_d,double theta1)
{
	vector<VectorXd> left_ik_available; //存放所有可用解的容器

	//调用获取中层反解函数，获得全部16组解
	vector<VectorXd> left_ik_8_values;
	left_ik_8_values = get8IkInBase(T_leftGripper_base_d, theta1);

	//获取未超限的解，放入容器
	for (int i = 0; i < left_ik_8_values.size(); ++i)
	{
		if (robotJointCheck(left_ik_8_values[i])) //16个解送入超限检查函数，没超限的话记录序号
		{
			left_ik_available.push_back(left_ik_8_values[i]);
		}
	}

	return left_ik_available;
}
//高层——关节角度迭代直至获取最优反解结果，geometry_msg类型位姿
//输入：左臂【夹爪】相对【base_link】的geometry_msg类型位姿，存放反解结果的容器
//输出：若未超限，输出左臂的一组最优反解关节角（从theta的初始offset处开始计算的角度）
//返回：反解成功/失败
bool RobotKinematicsSolver::getBestIkFromPoseInBase(geometry_msgs::Pose left_gripper_pose, VectorXd &left_ik_value)
{
	//把输入的geometry_msgs::Pose类型的位姿转换为位置向量和旋转矩阵
	Vector3d t_leftGripper_base_d;
	t_leftGripper_base_d << left_gripper_pose.position.x, left_gripper_pose.position.y, left_gripper_pose.position.z;

	//利用Eigen把四元数转旋转矩阵
	//注意Eigen四元数的赋值顺序是WXYZ，输出顺序是XYZW
	Quaterniond q(left_gripper_pose.orientation.w, left_gripper_pose.orientation.x, left_gripper_pose.orientation.y,left_gripper_pose.orientation.z);
	Matrix3d R_leftGripper_base_d;
	R_leftGripper_base_d = q.toRotationMatrix();

	//组合为抓手相对base的齐次变换矩阵
	Matrix4d T_leftGripper_base_d;
	T_leftGripper_base_d << R_leftGripper_base_d, t_leftGripper_base_d, 0, 0, 0, 1;

	//粒子群算法求最优关节角(暂无)
	double theta1_best[1];

	PSO(theta1_best,1,T_leftGripper_base_d);
	printf("全局最优位置:\n");
    for (int i = 0; i < 1; i++)
    {
        printf("%f ", theta1_best[i]);
        if (i > 0 && i % 7 == 0)
            printf("\n");
    }
    cout << "------------------------------------" << endl;
	//验证最优关节角下是否有可行解(暂无)
	
	// //若无可行解，开始循环迭代直至找到解
	// double theta1 = -1.57;

	// vector<VectorXd> left_ik_available; //存放所有可用解的容器

	// for (int j = 0; ; ++j)
	// {
	// 	if (theta1 > 1.57)
	// 	{
	// 		break;
	// 	}

	// 	//调用获取中层反解函数，获得全部16组解
	// 	vector<VectorXd> left_ik_16_values;
	// 	left_ik_16_values = get8IkInBase(T_leftGripper_base_d, theta1);

	// 	//获取未超限的解，放入容器
	// 	for (int i = 0; i < left_ik_16_values.size(); ++i)
	// 	{
	// 		if (robotJointCheck(left_ik_16_values[i])) //16个解送入超限检查函数，没超限的话记录序号
	// 		{
	// 			left_ik_available.push_back(left_ik_16_values[i]);
	// 		}
	// 	}

	// 	theta1 = theta1 + 0.2;
	// }

	// // cout << "迭代可行解数：" << left_ik_available.size()<< endl;
	// if(left_ik_available.empty())
	// {
	// 	// cout << "未找到解！" << endl;
	// 	return false;  //return后后续代码不会执行
	// }
		
	// //在可行解中找出距离各关节中心点最近的解，作为最优解输出
	// //计算左臂关节转动限制中线值
	// VectorXd left_arm_center(7);
	// left_arm_center << 0, 0, 0, 0, 0, 0, 0; //中线值

	// //计算所有可行解与中线值的差值向量的模，存入向量
	// vector<double> diff_norm;
	// for(int i=0; i<left_ik_available.size(); ++i)
	// {
	// 	diff_norm.push_back((left_ik_available[i]-left_arm_center).norm()); //求差值向量的模，存入vector便于找到最小值索引
	// 	//cout << diff_norm[i] << endl; 
	// }

	// //找出最小值及其索引
	// vector<double>::iterator smallest = min_element(diff_norm.begin(), diff_norm.end());
	// int smallest_index = distance(diff_norm.begin(), smallest);

	// //将最小索引对应的关节角赋值输出
    // left_ik_value = left_ik_available[smallest_index];

	return true;
}



/*
输入x[]为粒子位置
dim，为每个粒子的位置参数个数，即一组解有多少个：7
目标函数：避开关节
*/
double RobotKinematicsSolver::f1(double theta1[], int dim,Matrix4d T_leftGripper_base_d)
{
	double z = 0;
	vector<VectorXd> left_ik_available;
	vector<VectorXd> left_ik_8_values;
	left_ik_8_values = get8IkInBase(T_leftGripper_base_d, theta1[0]);
	//获取未超限的解，放入容器
	for (int i = 0; i < left_ik_8_values.size(); ++i)
	{
		if (robotJointCheck(left_ik_8_values[i])) //16个解送入超限检查函数，没超限的话记录序号
		{
			left_ik_available.push_back(left_ik_8_values[i]);
		}
	}

	if(left_ik_available.empty())
	{
		 cout << "未找到解！" << endl;
		return false;  //return后后续代码不会执行
	}

	//在可行解中找出距离各关节中心点最近的解，作为最优解输出
	//计算左臂关节转动限制中线值
	VectorXd left_arm_center(7);
	left_arm_center << 0, 0, 0, 0, 0, 0, 0; //中线值

	//计算所有可行解与中线值的差值向量的模，存入向量
	vector<double> diff_norm;
	for(int i=0; i<left_ik_available.size(); ++i)
	{
		diff_norm.push_back((left_ik_available[i]-left_arm_center).norm()); //求差值向量的模，存入vector便于找到最小值索引
		//cout << diff_norm[i] << endl; 
	}

	//找出最小值及其索引
	vector<double>::iterator smallest = min_element(diff_norm.begin(), diff_norm.end());
	
	int smallest_index = distance(diff_norm.begin(), smallest);

	//将最小索引对应的关节角赋值输出
   // left_ik_value = left_ik_available[smallest_index];

	// for (int i = 0; i < dim; i++)
	// {
	// 	//z += (i + 20) * (x[i] - i) * (x[i] - i);
    //     z += x[i]*x[i];
	// }
	z = diff_norm[smallest_index];
	return z;
}


/*
best为全局最优位置
DIM为目标函数的输入参数总个数，也即每个粒子的位置参数总个数：7,  每个粒子是一组解   //NUM个粒子，DATA_SIZE个位置
*/
void RobotKinematicsSolver::PSO(double* best, int DIM, Matrix4d T_leftGripper_base_d)
{
	particle swarm[NUM];
    for (int i = 0; i < DIM; i++)//初始化全局最优
    {
        best[i] = randf * (xmax - xmin) + xmin; //随机生成xmin~xmax的随机数
    }
    //初始化全局最优目标函数值为一个非常大的值
    double gbestf = 100000000000000.0;

    for (int i = 0; i < NUM; i++)  //初始化粒子群
    {
        particle* p1 = &swarm[i];   //获取每一个粒子结构体的地址
        for (int j = 0; j < DIM; j++)
        {
            //随机生成xmin~xmax的随机数作为该粒子的位置参数
            p1->x[j] = randf * (xmax - xmin) + xmin;
        }
        p1->f = f1(p1->x, DIM,T_leftGripper_base_d);   //计算当前时刻的目标函数值
        p1->bestf = 100000000000000.0;  //初始化每个粒子的历史最优目标函数值为一个非常大的值
    }


    double* V = (double*)calloc(DIM, sizeof(double));
    const int cnt = 100;   //总共2000次迭代
    double w = 0.0025 / (cnt - 1);  //设置w初值，后面逐渐减小

    for (int t = 0; t < cnt; t++)   //cnt次迭代
    {
        for (int i = 0; i < NUM; i++)  //NUM个粒子
        {
            particle* p1 = &swarm[i];
            for (int j = 0; j < DIM; j++)   //计算速度，并更新位置
            {
                //w*(cnt-1-t)随着t的增加逐渐减小
                V[j] = w * (cnt - 1 - t) * V[j] + c1 * randf * (p1->bestx[j] - p1->x[j]) + c2 * randf * (best[j] - p1->x[j]);
                //V[j] = c1*randf*(p1->bestx[j] - p1->x[j]) + c2*randf*(best[j] - p1->x[j]);
                //限制速度的上下限
                V[j] = (V[j] < vmin) ? vmin : ((V[j] > vmax) ? vmax : V[j]);
                p1->x[j] = p1->x[j] + V[j];  //更新位置参数
            }


            p1->f = f1(p1->x, DIM, T_leftGripper_base_d);   //计算当前粒子的当前时刻目标函数值
            //如果当前粒子的当前时刻目标函数值小于其历史最优值，则替换该粒子的历史最优
            if (p1->f < p1->bestf)
            {
                for (int j = 0; j < DIM; j++)
                {
                    p1->bestx[j] = p1->x[j];
                }
                p1->bestf = p1->f;
            }


            //如果当前粒子的历史最优值小于全局最优值，则替换全局最优值
            if (p1->bestf < gbestf)
            {
                for (int j = 0; j < DIM; j++)
                {
                    best[j] = p1->bestx[j];
                }

                //这里有点不理解，为什么替换全局最优值之后要重新随机分布该粒子的位置参数？
                //如果没有这部分代码，整体优化效果就会很差
                for (int j = 0; j < DIM; j++)
                {
                    p1->x[j] = randf * (xmax - xmin) + xmin;
                }


                gbestf = p1->bestf;
                printf("t = %d, gbestf = %lf\n", t, gbestf);
            }
        }
    }

    free(V);
}