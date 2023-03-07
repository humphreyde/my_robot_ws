#ifndef ROSAUXILIARY_H
#define ROSAUXILIARY_H

#include <vector>
#include <ros/ros.h>
#include "ubt_core_msgs/JointCommand.h"

#include "StructDefine.h"

class RosAuxiliary
{
public:

    void pubPosData(ros::Publisher pub, const std::vector<double> data);

    void pubJoint(ros::Publisher pub, const double data);

    void pubUData(ros::Publisher pub, const double data);

};


#endif // ROSAUXILIARY_H
