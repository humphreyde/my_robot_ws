#ifndef DECAREINTERPOLATION_H
#define DECAREINTERPOLATION_H

#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Core>

#include "StructDefine.h"

class Decare_Inter
{
public:

    std::vector<InterpolationPara> GenerateInterPoints(std::vector<VelocityPlanPara> &velocityPlanPara);


    int JointInterpolation(VelocityPlanPara *velPlanPara, std::vector<std::vector<double>> *interPoints);


    int Decare_LInterpolation(VelocityPlanPara *velPlanPara, std::vector<std::vector<double>> *interPoints);


    std::vector<double> CalJointByTime(double time, PlanningParameter thePlanPara);
    std::vector<double> CalDecareByTime(double time, PlanningParameter thePlanPara, PlanningParameter thePlanParaSecond);
    std::vector<double> CalDecareByTime(double time, PlanningParameter thePlanParaFirst);


    void CalculateTransitPointNum(std::vector<InterpolationPara> &interPolationPara);

    Eigen::Quaterniond rpy2quaternion(Eigen::VectorXd rpy);


    Eigen::VectorXd quaternion2rpy(Eigen::Quaterniond quaternion);


    Eigen::Matrix3d quaternion2rotation(Eigen::Quaterniond quaternion);


    Eigen::Quaterniond rotation2quaternion(Eigen::Matrix3d rotationMatrix);


    Eigen::Matrix3d rpy2rotation(Eigen::VectorXd rpy);


    Eigen::VectorXd rotation2rpy(Eigen::Matrix3d rotationMatrix);

    Eigen::VectorXd slerp(Eigen::VectorXd starting, Eigen::VectorXd ending, double u);

};

#endif // DECAREINTERPOLATION_H
