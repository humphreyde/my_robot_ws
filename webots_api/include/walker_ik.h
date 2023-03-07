#ifndef WALKER_IK_H
#define WALKER_IK_H
#include<eigen3/Eigen/Eigen>
#include<vector>
#include<iostream>
#include"avoid_jointlimit.h"
using namespace Eigen;
namespace walkerArm_IK {
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
    class walkerArmIkSolver{
    public:
        VectorXd IK_joints_armq(VectorXd Pose, VectorXd joints, JointParameter *j);
        VectorXd IK_joints(VectorXd Pose);
        Matrix4d FK_joints(VectorXd q);
        double calculate_armq(Vector3d last_position,VectorXd joint_angles);
        void jointParameterInit_right(JointParameter *j, double SAFE_LIMITED_POSITION);
        void jointParameterInit_left(JointParameter *j, double SAFE_LIMITED_POSITION);
    private:
        double determind_armq(Matrix3d rotation_matrix,Matrix3d R04);
        VectorXd ikSolverAnalytical7DOF( const double * Pose, Matrix3d rotation_matrix, Vector3d last_position, VectorXd joints, JointParameter *left_Param);
        VectorXd calculate_joints(Matrix3d As, Matrix3d Bs, Matrix3d Cs, Matrix3d Aw, Matrix3d Bw, Matrix3d Cw, double arm_q);
        double last_arm;
        bool flag_first = false;
        double JNT_LIMITS_LOW[7];
        double JNT_LIMITS_UP[7];
        KalmanState K_q;
    };
    VectorXd leftArmIk(VectorXd Pose, VectorXd joints, JointParameter *j,walkerArmIkSolver leftArm);
    VectorXd rightArmIk(VectorXd Pose, VectorXd joints, JointParameter *j, walkerArmIkSolver rightArm);
}
#endif
