#ifndef STRUCT_DEFINE_H
#define STRUCT_DEFINE_H

#include <vector>

const int RobotDOF = 7;
const int PoseDIM = 7;
const double InterpolationCycle = 0.002;

const double Acc_Limitation = 3.0;                 //acceleration
const double Jerk_Limitation = 10.0;              //jerk
const double Joint_Acc_Limitation = 3.0;           //joint acceleration
const double Joint_Jerk_Limitation = 10.0;        //joint jerk


typedef enum
{
    Joint = 1,        //joint interpolation
    Decare_L = 2,     //Cartesian interpolation
}EnumCommand;


typedef struct
{
  int commandNum;
  int commandName;
  int next;
  int waypointNum;
  bool isDecare;
  std::vector<std::vector<double>> waypoints;
  double set_vel;
  double start_vel;
  double end_vel;
  double transformPrecision;

}UserToRobotCommand;


typedef struct
{
    double x;
    double y;
    double z;

}Point3D;


typedef struct
{
  double length;

  double startVel;
  double constVel;
  double endVel;

  //the time of each segment
  double T1;
  double T2;
  double T3;
  double T4;
  double T5;
  double T6;
  double T7;

  double totalTime;

  //the velocity of each segment
  double vel1;
  double vel2;
  double vel3;
  double vel4;
  double vel5;
  double vel6;
  double vel7;

  //the distance of each segment
  double dis1;
  double dis2;
  double dis3;
  double dis4;
  double dis5;
  double dis6;
  double dis7;

  double acc;
  double dec;

  //actual jerk
  double jerk1;
  double jerk3;
  double jerk5;
  double jerk7;

  double jerk;

}SPlanPara;


typedef struct
{
  int commandNum;
  int commandName;
  int next;
  int waypointNum;
  std::vector<std::vector<double>> waypoints;
  Point3D center;
  double radius;

  //cordinate transform parameter
  double u[3];
  double v[3];
  double w[3];

  double StartAngle;
  double MidAngle;
  double EndAngle;

  double RxPara[3];
  double RyPara[3];
  double RzPara[3];

  int index;
  double transformPrecision;
  double cur_U;

  SPlanPara sPlanPara;

  int isFinish;

}VelocityPlanPara;


typedef struct
{
  int commandName;
  double transformPrecision;
  int remainPointNum;
  int curToLastPointNum;
  int curToNextPointNum;

  std::vector<std::vector<double>> interpolationVector;
  bool isDecare;
  int commandNum;

}InterpolationPara;

/*--------------------------------------PlanByTime call---------------------------------*/
typedef struct
{
    int flag;
    int cPoint_num;
    double duration;
    double *cqi;
    double *cti;

    double *ai;
    double *bi;
    double *ci;
    double *di;

    double *h;

    double *A;
    double *B;
    double *C;
    double *D;
    double *E;
    double *M;

}CubicSplineIterpolation;

typedef struct
{
    int len;       //number of waypoints
    double *q;     //waypoint joint position of single joint
    double *po;    //waypoint pose infomation of single joint
    double *jt;    //time correspond to each waypoint of single joint

}CSIJoint;

typedef struct
{
  CubicSplineIterpolation *CSI;  //CSI parameter of single joint
  CSIJoint *Joints;              //waypoint infomation of single joint
  int num;                       //dof of robot
  int dim;                       //represent the (xyz xyzw), count for 7

}CSIList;

/*-----------------------------------------------------------------------------------------*/

typedef struct
{
    //the length of path
    double length;

    //setting velocity for start-running-end
    double velStart;
    double velConst;
    double velEnd;

    //the time of each segment
    double T1;
    double T2;
    double T3;
    double T4;
    double T5;
    double T6;
    double T7;

    double totalTime;

    //the velocity of each segment
    double vel1;
    double vel2;
    double vel3;
    double vel4;
    double vel5;
    double vel6;
    double vel7;

    //the distance of each segment
    double dis1;
    double dis2;
    double dis3;
    double dis4;
    double dis5;
    double dis6;
    double dis7;

    //setting acceleration and deceleration
    double acc;
    double dec;

    //actual jerk
    double jerk1;
    double jerk3;
    double jerk5;
    double jerk7;

    //setting jerk
    double jerk;

}SingleSPara;


typedef struct
{
    std::vector<std::vector<double>> waypoints;

    double remainTime;
    double curToLastTime;
    double curToNextTime;

    double transitFlag;

    bool isDecare;

    SingleSPara theSingleSPara;

}PlanningParameter;   //single segment parameter

#endif //STRUCT_DEFINE_H
