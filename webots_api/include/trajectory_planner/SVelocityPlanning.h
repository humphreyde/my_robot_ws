#ifndef S_VELOCITY_PLANNING_H
#define S_VELOCITY_PLANNING_H

#include "StructDefine.h"


/***********************************************************
    * Class Name: SVelocityPlanner
    * Description: Used for S velocity plan
\**********************************************************/
class SVelocityPlanner
{
public:
    //public member function
    void initADcc();

    void initJointSPara(VelocityPlanPara *velPara);
    void initDecare_LSPara(VelocityPlanPara *velPara);

    void checkJerkCondition();

    void checkAccCondition();

    void checkDecCondition();

    int checkTotalLenCondition();

    void updateADcc();

    void updateVelocity();

    double cal_UByPosition(double J, double A, double f, double Tm, double u0);

    double cal_UByVelocity(double J, double A, double f, double Tm);

    int cal_SPara(VelocityPlanPara *velPlanPara);

    int getTotalNumPara();

    double getLength();

    double cal_UForLine(int index);
    double cal_UForLine(double timeStamp, const SingleSPara &thePara);

    SVelocityPlanner();
    SVelocityPlanner(const SVelocityPlanner &cp);

    ~SVelocityPlanner();

private:
    //InterpolationCycle
    double m_interpolationCycle;
    
    //total length of the path
    double m_length;

    //setting velocity for start-running-end
    double m_velStart;
    double m_velConst;
    double m_velEnd;

    //the time of each segment
    double m_T1;
    double m_T2;
    double m_T3;
    double m_T4;
    double m_T5;
    double m_T6;
    double m_T7;

    //the interpolation points number of each segment
    int m_Num1;
    int m_Num2;
    int m_Num3;
    int m_Num4;
    int m_Num5;
    int m_Num6;
    int m_Num7;

    //Num12 = Num1 + Num2
    int m_Num12;
    int m_Num13;
    int m_Num14;
    int m_Num15;
    int m_Num16;

    //total interpolation points number
    int m_TotalNum;

    //the velocity of each segment
    double m_vel1;
    double m_vel2;
    double m_vel3;
    double m_vel4;
    double m_vel5;
    double m_vel6;
    double m_vel7;

    //the distance of each segment
    double m_dis1;
    double m_dis2;
    double m_dis3;
    double m_dis4;
    double m_dis5;
    double m_dis6;
    double m_dis7;

    //setting acceleration and deceleration
    double m_acc;
    double m_dec;

    //actual jerk
    double m_jerk1;
    double m_jerk3;
    double m_jerk5;
    double m_jerk7;

    //setting jerk
    double m_jerk;
};

#endif //S_VELOCITY_PLANNING_H
