#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <Eigen/Dense>
#include <sys/time.h>
#include <geometry_msgs/Pose.h>
#include "robot_kinematics_solver/robot_kinematics_solver.h"

#include <chrono>
using namespace chrono;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_api_node");
    ros::NodeHandle n("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //定义一个求解器对象
    RobotKinematicsSolver solver;

    //计时变量
    auto start = system_clock::now();

    /*************末端正解测试*****************/
    //左臂预期关节角赋值
    VectorXd left_q_expect(7);
    left_q_expect << 0,0.5, 0, 0.5, 0, 0, 0;

    //获取左臂末端相对base_link的正解位姿
    geometry_msgs::Pose left_fk_pose;
    Matrix4d left_fk_T;

    left_fk_pose = solver.getEEPoseInBase(left_q_expect);
    left_fk_T = solver.getEETransMatInBase(left_q_expect);
    

    auto end   = system_clock::now();
    auto duration = duration_cast<microseconds>(end - start);
    cout <<  "1. 左臂正解耗时：" << double(duration.count()) * microseconds::period::num   << " us" << endl;
    cout << "2. 左臂末端相对base_link的正解位姿：\n" << left_fk_T << endl;
    cout << "------------------------------------------" << endl;


    /***************反解*****************/
    vector<VectorXd> left_ik_values;

    start = system_clock::now();

    // Vector3d t_leftGripper_base_d;
	// t_leftGripper_base_d << left_fk_pose.position.x, left_fk_pose.position.y, left_fk_pose.position.z;

    // Quaterniond q(left_fk_pose.orientation.w, left_fk_pose.orientation.x, left_fk_pose.orientation.y,left_fk_pose.orientation.z);
	// Matrix3d R_leftGripper_base_d;
	// R_leftGripper_base_d = q.toRotationMatrix();

    // Matrix4d T_leftGripper_base_d;
	// T_leftGripper_base_d << R_leftGripper_base_d, t_leftGripper_base_d, 0, 0, 0, 1;

   // left_single_ik_values = solver.getLeft16IkFromPoseInBase(T_leftGripper_base_d, 0.8);
    left_ik_values = solver.getLimitIkInBase(left_fk_T,0);

    end   = system_clock::now();
    duration = duration_cast<microseconds>(end - start);
    cout <<  "3. 8组解中可行的反解耗时：" << double(duration.count()) * microseconds::period::num << " us" << endl;

    cout <<  "4. 8组解中可行的反解个数：" << left_ik_values.size() << endl;
    
    for (int i = 0; i < left_ik_values.size(); ++i)
	{
        cout <<"5. 反解的关节角度值：" <<left_ik_values[i].transpose() << endl;
        printf("6. 反解值正解结果：\n");
        Matrix4d left_ikfk_T = solver.getEETransMatInBase(left_ik_values[i]);
         cout << left_ikfk_T << endl;
         cout << "------------------------------------" << endl;
	}
	

    // int DIM = 7;//维数
    // double gbestx[7];//全局最优位置
    // solver.PSO(gbestx, DIM);
    // printf("全局最优位置:\n");
    // for (int i = 0; i < DIM; i++)
    // {
    //     printf("%f ", gbestx[i]);
    //     if (i > 0 && i % 7 == 0)
    //         printf("\n");
    // }
    // cout << "------------------------------------" << endl;




    /*************【最优】反解测试：用正解结果测试反解*****************/
    VectorXd left_ik_value;

    start = system_clock::now();

    bool success;
    success = solver.getBestIkFromPoseInBase(left_fk_pose, left_ik_value);

    end   = system_clock::now();
    duration = duration_cast<microseconds>(end - start);
    cout <<  "5. 左臂最优反解耗时：" << double(duration.count()) * microseconds::period::num << " us" << endl;

    printf("5. 左臂最优反解值：\n");
    cout << left_ik_value.transpose() << endl;

    printf("5. 左臂最优反解值正解结果：\n");
    geometry_msgs::Pose left_ikfk_pose = solver.getEEPoseInBase(left_ik_value);
    cout << left_ikfk_pose << endl;

    printf("5. 左臂最优反解值正解结果与输入值的误差：\n");
    VectorXd left_ikfk_err(7);
    left_ikfk_err << (left_ikfk_pose.position.x - left_fk_pose.position.x),
            (left_ikfk_pose.position.y - left_fk_pose.position.y),
            (left_ikfk_pose.position.z - left_fk_pose.position.z),
            (left_ikfk_pose.orientation.x - left_fk_pose.orientation.x),
            (left_ikfk_pose.orientation.y - left_fk_pose.orientation.y),
            (left_ikfk_pose.orientation.z - left_fk_pose.orientation.z),
            (left_ikfk_pose.orientation.w - left_fk_pose.orientation.w);
    cout << left_ikfk_err.transpose() << endl;
    // cout << "--------------------" << endl;

    // VectorXd right_ik_value;

    //     right_fk_pose.position.x = 0.323632;
    // right_fk_pose.position.y = 0.0732591;
    // right_fk_pose.position.z = -0.25;
    // right_fk_pose.orientation.x = 0;
    // right_fk_pose.orientation.y =  0;
    // right_fk_pose.orientation.z =  0;
    // right_fk_pose.orientation.w = 1;

    // gettimeofday(&timeStart, NULL); //获取当前绝对时间
    // success = solver.getRightBestIkFromPoseInBase(right_fk_pose, right_ik_value);
    // gettimeofday(&timeNow, NULL); //获取当前绝对时间
    // t = (timeNow.tv_sec - timeStart.tv_sec) + (timeNow.tv_usec - timeStart.tv_usec)/1000.0; //计算算法耗时
    // printf("6. 右臂最优反解耗时：【%.3fms】\n", t);

    // printf("6. 右臂最优反解值：\n");
    // cout << right_ik_value.transpose() << endl;

    // printf("6. 右臂最优反解值正解结果：\n");
    // geometry_msgs::Pose right_ikfk_pose = solver.getRightEEPoseInBase(right_ik_value);
    // cout << right_ikfk_pose << endl;

    // printf("6. 右臂最优反解值正解结果与输入值的误差：\n");
    // VectorXd right_ikfk_err(7);
    // right_ikfk_err << (right_ikfk_pose.position.x - right_fk_pose.position.x),
    //         (right_ikfk_pose.position.y - right_fk_pose.position.y),
    //         (right_ikfk_pose.position.z - right_fk_pose.position.z),
    //         (right_ikfk_pose.orientation.x - right_fk_pose.orientation.x),
    //         (right_ikfk_pose.orientation.y - right_fk_pose.orientation.y),
    //         (right_ikfk_pose.orientation.z - right_fk_pose.orientation.z),
    //         (right_ikfk_pose.orientation.w - right_fk_pose.orientation.w);
    // cout << right_ikfk_err.transpose() << endl;

    cout << "--------------------" << endl;

    return 0;
}
