#ifndef AUXILIARYFUNCTION_H
#define AUXILIARYFUNCTION_H

#include <iostream>
#include <vector>
#include <string>

#include"StructDefine.h"


class AuxiliaryFunction
{
public:

    std::vector<UserToRobotCommand> ParseCommand(CSIList *points, double velocity, std::string commandName);


    void GenerateInterCmd(const std::vector<UserToRobotCommand> &userCmd, std::vector<VelocityPlanPara> *interCmd);
};

#endif // AUXILIARYFUNCTION_H
