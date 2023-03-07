#ifndef TRAJECTORYPLANNER_H
#define TRAJECTORYPLANNER_H

#include <string>

#include "StructDefine.h"
#include "PlanAuxiliary.h"
#include "DecareInterpolation.h"

class TrajectoryPlanner
{
public:
    double InitInterpolation(CSIList *points, double velocity, std::string commandName);

    std::vector<double> CalInterpolation(double time, std::vector<double> lastJointPosition);

private:
    AuxiliaryFunction theAuxiliaryFunction;
    Decare_Inter theDecare_Inter;

    std::vector<PlanningParameter> m_thePlanPara;
    double m_duration;
    std::vector<double> m_timeList;
};


#endif // TRAJECTORYPLANNER_H
