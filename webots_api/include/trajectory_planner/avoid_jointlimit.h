#ifndef AVOID_JOINTLIMIT_H
#define AVOID_JOINTLIMIT_H

#include <stdlib.h>
#include <vector>
#include <iostream>
#include<eigen3/Eigen/Eigen>

typedef struct
{
    std::string name;
    std::string type;
    double min_angle;
    double max_angle;
    double init_angle;
    double max_speed;
    double max_effort;
    int direction;
    double offset;
    int reduce;

    double mid_angle;
    double range;

    double current_angle;
    double current_velocity;
    double current_effort;

    double command_angle;

}JointParameter;

void jointParameterInit(JointParameter *j, int joint_num);
void jointParameterInit_right(JointParameter *j);
void jointParameterInit_left(JointParameter *j);
Eigen::VectorXd JointLimit_Grad(JointParameter *j, Eigen::VectorXd q);

#endif
