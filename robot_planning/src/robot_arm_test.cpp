/*
手臂测试
*/


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>

//Eigen头
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <algorithm>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Trigger.h>

//geometry_msgs头
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/WrenchStamped.h>


//actionlib头
#include <actionlib/client/simple_action_client.h>

//control_msgs头
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
//trajectory_msgs头
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "baxter_core_msgs/JointCommand.h"
#include "robot_planning/robot_planning.h"
#include "robot_planning/robot_kinematics_solver.h"


using namespace std;
using namespace Eigen;
using namespace KDL;

/*********************************全局变量**************************************/
//关节相关订阅发布
ros::Subscriber robot_state_sub;
ros::Publisher robot_controller_pub;

ros::Subscriber lhand_force_sub;

//目标物位置订阅
ros::Subscriber bottle_pos_sub;
ros::Subscriber cup_pos_sub;
ros::Subscriber torso_aruco_sub;

ros::Publisher speed_pub;

//任务接受服务器
ros::ServiceServer grasp_service, opendoor_service;



bool leftarm_state_got = false;  //当订阅到 关节状态 时为true


bool grasp_task_start = false; //任务启动标记,接收到call后true,任务执行完后变false
bool opendoor_task_start = false;


VectorXd joint_state = VectorXd::Zero(7); //存储当前左臂关节角度值的向量


geometry_msgs::Pose bottle_pose, cup_pose;
geometry_msgs::PoseStamped aruco_pose;


vector<string> right_arm_joint_names; //存放手臂关节名称的数组
vector<double> right_arm_goal_position;

RobotKinematicsSolver solver;
/*********************************回调函数***********************************/
//左臂关节状态回调函数
void leftArmJointStates_Callback(const sensor_msgs::JointState & msg)
{
    for(int i=0;i<7;i++)
    {       
        joint_state(i) = msg.position[i];        
    }
    // cout << "joint_state:" << joint_state.transpose()<< endl;

    leftarm_state_got = true;
}




//左臂单独运动封装，减少代码量
bool moveLeftArm(MatrixXd left_waypoints, VectorXd durations)
{
    if(left_waypoints.rows() - 1 != durations.rows())
    {
        cout << "路径点与时间段不匹配！" << endl;
        return false;
    }

    struct timeval timeStart, timeNow; //绝对起始时间、绝对当前时间
    double t = 0;                     //从timeStart开始的相对时间,t = timeNow - timeNow
    VectorXd q_t(7);            //存放规划出来的、t时刻左臂关节角位置的变量

    gettimeofday(&timeStart, NULL); //获取当前绝对时间
    t = 0; //当前相对时间置0
    while(ros::ok() && (t < durations.sum())) 
    {
        gettimeofday(&timeNow, NULL); //获取当前绝对时间
        t = (timeNow.tv_sec - timeStart.tv_sec) + (timeNow.tv_usec - timeStart.tv_usec)/1000000.0; //更新当前相对时间
//        cout<<"当前时间："<<t<<endl;
        //规划t时刻各关节角度
        q_t = multipoints_trajectory_plan(left_waypoints, durations, t);
        //发送规划角度到控制器
        robotPositionController(robot_controller_pub, q_t);
        cout<<q_t.transpose()<<endl;
        ros::spinOnce();
        ros::Duration(0.02).sleep();
    }
    cout << "左臂到达！" << endl;

    // Matrix4d left_fk_T;
	// geometry_msgs::Pose fk_pose;
	// fk_pose = solver.getEEPoseInBase(q_t); 
	// left_fk_T = solver.getEETransMatInBase(q_t);
  	// cout << "1. 左臂末端相对base_link的正解位姿：\n" << left_fk_T << endl;
	// cout << "2. 左臂末端相对base_link的正解位姿：\n" << fk_pose << endl;

    return true;

}


/******************************************主函数******************************************/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_arm_test");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();

    //订阅与发布

    robot_state_sub = n.subscribe("/robot/joint_states", 1000, &leftArmJointStates_Callback);
    robot_controller_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/controller", 1000);

    ros::Duration(1.0).sleep(); //等待话题订阅


    //DEBUG：是否开启手臂运动
    bool is_motion = true;
    bool is_sim = false;
    
    // 测试是否订阅关节角
   while(!leftarm_state_got)
   {
       ros::spinOnce();
       ros::Duration(0.001).sleep();
       cout<<"等待关节角数据..."<<endl;
   }
    
    if(is_motion)
    {
        double x0,y0,z0,qw0,qx0,qy0,qz0;
        //0,-0.3,0,-0.4,0,-0.87,0,的正解位姿
        x0 = 0.449423;
        y0 = 0.947459;
        z0 = 0;
        qw0 = 0.500199;
        qx0 = 0.500199;
        qy0 = 0.499801;
        qz0 = 0.499801;
        cout <<"前向测试..." << endl;
        
        // Vector3d t_leftGripper_base_d;
        // t_leftGripper_base_d <<x,y,z;

        // Quaterniond q(qw,qx,qy,qz);
        // Matrix3d R_leftGripper_base_d;
        // R_leftGripper_base_d = q.toRotationMatrix();

        // Matrix4d T_leftGripper_base_d;
        // T_leftGripper_base_d << R_leftGripper_base_d, t_leftGripper_base_d, 0, 0, 0, 1;
        double theta = 0,r = 0.05;
        double y[100],z[100];
        for(int i = 0; i<100; i++)//100个点,x和姿态不变，y，z变化
        {
            y[i] = y0+r*cos(theta);
            z[i] = z0+r*sin(theta);
            theta += M_PI/50;

        }

        // cout <<"inverse kinematics..." << endl;
        // VectorXd qInverse(7);
        // vector<VectorXd> qInverse100;
        // for(int i = 0; i<100; i++){

        //     geometry_msgs::Pose left_gripper_pose;
		
        //     //位置可以直接赋值
        //     left_gripper_pose.position.x = x0; //取1行4列
        //     left_gripper_pose.position.y = y[i]; //取1行4列
        //     left_gripper_pose.position.z = z[i]; //取1行4列
        //     cout <<"z[i]"<<  left_gripper_pose.position.z<< endl;
        //     cout <<"y[i]"<<   y[i]<< endl;

        //     //姿态赋值
        //     left_gripper_pose.orientation.x = qx0; //务必注意Eigen四元数的储存顺序
        //     left_gripper_pose.orientation.y = qy0;
        //     left_gripper_pose.orientation.z = qz0;
        //     left_gripper_pose.orientation.w = qw0;
		
        //     solver.getBestIkFromPoseInBase(left_gripper_pose,qInverse);
        //     qInverse100.push_back(qInverse);
            
        // }
        // cout <<"inverse kinematics finished..." << endl;

        MatrixXd waypoints_left0(3,7); 
        VectorXd t_0(2); //t_应比waypoints少1
        for(int i = 0; i<t_0.size(); i++){
        t_0(i) = 20;}

	//	for(int i = 0; i<2; i++){
        waypoints_left0 <<  0, 0, 0, 0, 0, 0, 0,
                        -15,-20,30,-30,-60,-60,-60,
                          //-M_PI/12, -M_PI/6, M_PI/6,-M_PI/3,-M_PI/3,-M_PI/3,-M_PI/3,
                          0, 0, 0, 0, 0, 0, 0;
        // waypoints_left0.block<1, 7>(i, 0) =  qInverse100[i].transpose();
						//	
                          //  0.5235987, 0.000000,  0.5235987, 1.0471975, 1.0471975, 1.0471975,1.0471975;						 
                         
                        // 0,-0.3,0,-0.4,0,-0.87,0;   
                        // qInverse(0), qInverse(1),qInverse(2),qInverse(3),qInverse(4),qInverse(5),qInverse(6),
                        // 0,-0.4,0,-0.3,0,-0.87,0;
						  // 0,-0.2,-0,-03,0,-0.5,0;

      //  }                 
                            
        //修改路径点第一点为当前位姿关节角,注意执行此操作前必须经过一轮spin
        ros::spinOnce();
        ros::Duration(0.01).sleep();
        for(int i=0;i<7;i++)
        {
            waypoints_left0(0,i) = joint_state(i);
        }
        cout << "joint_state_test:" << waypoints_left0<< endl; 
        moveLeftArm(waypoints_left0, t_0);
    }
    
    return 0;

}
