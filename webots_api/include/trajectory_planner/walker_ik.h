#ifndef WALKER_IK_H
#define WALKER_IK_H

#include <eigen3/Eigen/Eigen>
#include <vector>
#include <iostream>
#include <string>

#include"avoid_jointlimit.h"

struct KalmanState{
    double x[2];     /* state: [0]-angle [1]-diffrence of angle, 2x1 */
    double A[2][2];  /* X(n)=A*X(n-1)+U(n),U(n)~N(0,q), 2x2 */
    double H[2];     /* Z(n)=H*X(n)+W(n),W(n)~N(0,r), 1x2   */
    double q[2];     /* process(predict) noise convariance,2x1 [q0,0; 0,q1] */
    double r;        /* measure noise convariance */
    double p[2][2];  /* estimated error convariance,2x2 [p0 p1; p2 p3] */
    double gain[2];  /* 2x1 */
};

struct DHParameter
{
    std::string name;
    double offset;
    double theta;
    double d;
    double a;
    double alpha;
    double L;
};

class IkSolver{
public:
    //IK
    Eigen::VectorXd IK_joints_armq(Eigen::VectorXd Pose, Eigen::VectorXd joints, JointParameter *j);
    Eigen::VectorXd IK_joints(Eigen::VectorXd Pose);

    //FK
    Eigen::Matrix4d FK_joints(Eigen::VectorXd q);
    double calculate_armq(Eigen::Vector3d last_position, Eigen::VectorXd joint_angles);

private:
    double determind_armq(Eigen::Matrix3d rotation_matrix, Eigen::Matrix3d R04);
    Eigen::VectorXd ikSolverAnalytical7DOF(const double * Pose, Eigen::Matrix3d rotation_matrix, Eigen::Vector3d last_position, Eigen::VectorXd joints, JointParameter *left_Param);
    Eigen::VectorXd calculate_joints(Eigen::Matrix3d As, Eigen::Matrix3d Bs, Eigen::Matrix3d Cs, Eigen::Matrix3d Aw, Eigen::Matrix3d Bw, Eigen::Matrix3d Cw, double arm_q);
    double judge_last(double current, double last, double threshold);
    double last_arm;
    double last_j[7];
    bool flag_first = false;
    double JNT_LIMITS_LOW[7];
    double JNT_LIMITS_UP[7];
    double last_JNT_LIMITSq6, last_JNT_LIMITSq7;
    KalmanState K_q;
};


#endif
