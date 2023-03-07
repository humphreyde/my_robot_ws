#ifndef WEBOTS_H
#define WEBOTS_H

#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include <map>
#include <cfloat>
#include <boost/system/system_error.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Range.h>
#include "geometry_msgs/Point.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// #include <webots_api/SceneSelection.h>

#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/position_sensor.h>
#include <webots/utils/motion.h>
#include <webots/touch_sensor.h>

#include <webots/inertial_unit.h>
#include <webots/accelerometer.h>
#include <webots/gyro.h>
#include <webots/gps.h>
#include <webots/supervisor.h>

#include <webots/camera.h>
#include <webots/range_finder.h>
#include <webots/distance_sensor.h>
/**
 * @namespace webots_api
 */
namespace webots_api{



    typedef enum {
        LShoulderPitch = 0,
        LShoulderRoll,
        LShoulderYaw,
        LElbowRoll,
        LElbowYaw,        
        LWristPitch,
        LWristRoll,
    }ArmJointName;


#define MOTOR_NUMBERS 7

    /**
     * @class WebotsApi
     */
    class WebotsApi{
    public:
        /**
         * @brief constructor
         */
        WebotsApi();

        /**
         * @brief destructor
         */
        ~WebotsApi();


        /**
         * @brief Motor Read Api
         * @param msg_out: measured encoder info
         */

      //  bool WheelApiRead(double &msgs_out);
        bool ArmJointApiRead(sensor_msgs::JointState &msgs_out, const int mode);


        /**
         * @brief motor write joint data
         * @param mode
         * @param msgs_in
         * @return
         */

        bool LeftArmJointApiWrite(const sensor_msgs::JointState msgs_in, const int mode);

        /**
         * @brief Sensor Api
         * @return
         */

        bool RobotPosition(geometry_msgs::Point &msg_out);



        //robot pose read
        bool PoseOfRobotRead(geometry_msgs::PoseWithCovarianceStamped &msgs_out);

        //robot set position
        bool SetPositionOfRobot(const double x, const double y, const double z,
                                const double rx, const double ry, const double rz, const double theta);

        /**
         * @brief Api init
         * @return
         */
        bool ApiInit();
        bool CameraApiInit();
        bool cameraLeftExtraRGB(sensor_msgs::Image &msgs_out);

        double LTouchForce[6],RTouchForce[6];
        double time_step;
        // simulated devices
        //motor
        //Wheel
        WbDeviceTag LeftWheelMotor,RightWheelMotor;
        //head
        WbDeviceTag HeadRMotor,HeadLMotor;
        //arm
        WbDeviceTag RShoulderPitchMotor,RShoulderRollMotor,RShoulderYawMotor,RElbowRollMotor,RElbowYawMotor,RWristPitchMotor,RWristRollMotor;
        WbDeviceTag LShoulderPitchMotor,LShoulderRollMotor,LShoulderYawMotor,LElbowRollMotor,LElbowYawMotor,LWristPitchMotor,LWristRollMotor;
        //hand
        WbDeviceTag RFirstFinger1Motor_1,RFirstFinger2Motor,RSecondFinger1Motor,RSecondFinger2Motor,RThirdFinger1Motor,RThirdFinger2Motor,
                    RFourthFinger1Motor,RFourthFinger2Motor,RFifthFinger1Motor,RFifthFinger2Motor;
        WbDeviceTag LFirstFinger1Motor_1,LFirstFinger2Motor,LSecondFinger1Motor,LSecondFinger2Motor,LThirdFinger1Motor,LThirdFinger2Motor,
                    LFourthFinger1Motor,LFourthFinger2Motor,LFifthFinger1Motor,LFifthFinger2Motor;

        //joint sensor
        //wheel
        WbDeviceTag LeftWheelSensor,RightWheelSensor;
        //head
        WbDeviceTag HeadRSensor,HeadLSensor;
        //arm
        WbDeviceTag RShoulderPitchSensor,RShoulderRollSensor,RShoulderYawSensor,RElbowRollSensor,RElbowYawSensor,RWristPitchSensor,RWristRollSensor;
        WbDeviceTag LShoulderPitchSensor,LShoulderRollSensor,LShoulderYawSensor,LElbowRollSensor,LElbowYawSensor,LWristPitchSensor,LWristRollSensor;
        WbDeviceTag RFirstFinger1Sensor_1,RFirstFinger2Sensor,RSecondFinger1Sensor,RSecondFinger2Sensor,RThirdFinger1Sensor,RThirdFinger2Sensor,
                    RFourthFinger1Sensor,RFourthFinger2Sensor,RFifthFinger1Sensor,RFifthFinger2Sensor;
        WbDeviceTag LFirstFinger1Sensor_1,LFirstFinger2Sensor,LSecondFinger1Sensor,LSecondFinger2Sensor,LThirdFinger1Sensor,LThirdFinger2Sensor,
                    LFourthFinger1Sensor,LFourthFinger2Sensor,LFifthFinger1Sensor,LFifthFinger2Sensor;

        //Force & Torque sensor
        WbDeviceTag LTouchSensor,RTouchSensor;//no use now
        WbDeviceTag LFootSensor, RFootSensor;//no use now
        //force sensor
        WbDeviceTag LFoot6Axis, RFoot6Axis, LHand6Axis, RHand6Axis;
        //torque sensor
        WbDeviceTag LFootTorqueX,LFootTorqueY,LFootTorqueZ,RFootTorqueX,RFootTorqueY,RFootTorqueZ,
                    LHandTorqueX,LHandTorqueY,LHandTorqueZ,RHandTorqueX,RHandTorqueY,RHandTorqueZ;

        //IMU sensor
        //body
        WbDeviceTag IMU_Angle,IMU_AngleVelocity,IMU_Acceleration;
        //camera
        //front
        WbDeviceTag Boteye_F_Angle,Boteye_F_AngleVelocity,Boteye_F_Acceleration;
        //back
        WbDeviceTag Boteye_B_Angle,Boteye_B_AngleVelocity,Boteye_B_Acceleration;
        //head camera imu
        WbDeviceTag Head_Angle,Head_AngleVelocity,Head_Acceleration;

        //camera

        WbDeviceTag Bottom_RGB, Head_RGB, Left_Extra_RGB,Right_Extra_RGB;
        WbDeviceTag Bottom_Depth, Head_Depth, Left_Extra_Depth,Right_Extra_Depth;
        WbDeviceTag Left_F_Eye,Right_F_Eye,Left_B_Eye,Right_B_Eye,Left_Eye, Right_Eye;
        sensor_msgs::Image imageBottomRGB_, imageBottomDepth_,
        imageLeftExtraRGB_,imageLeftExtraDepth_,imageRightExtraRGB_,imageRightExtraDepth_,
        imageTopRGB_, imageTopDepth_, imageFrontLeftRGB_, imageFrontRightRGB_,
        imageBackLeftRGB_, imageBackRightRGB_, imageHeadRGB_, imageHeadDepth_,imageRightRGB_,imageLeftRGB_;

        //ultrasound
        WbDeviceTag DistanceLeftFront, DistanceRightFront, DistanceLeftBack, DistanceRightBack;
        sensor_msgs::Range UltraSoundLeftFront_, UltraSoundRightFront_,
        UltraSoundLeftBack_, UltraSoundRightBack_;

        //position from simulation
        WbDeviceTag WalkerGPS;
        WbNodeRef robot_node, ankle_node, LKnee_node, LAnklePitch_node;
        WbFieldRef trans_field, ankle_trans_field, LKnee_trans_field, LAnklePitch_trans_field, pose_field;

        double DesiredPosition[MOTOR_NUMBERS][2];
        double DesiredVelocity[MOTOR_NUMBERS][2];
        double DesiredTorque[MOTOR_NUMBERS][2];
        double FeedbackPosition[MOTOR_NUMBERS][2];
        double FeedbackVelocity[MOTOR_NUMBERS][2];
        double JointPID[MOTOR_NUMBERS][3];

        double *L6AxisForce, *R6AxisForce;
        double LFootTorque[3],RFootTorque[3];

        double *IMUAngle,*IMUAngleVelocity,*IMUAcceleration;

        //position from simulation
        double *PosGPS;
        double PosSim[3];
        double PosAnkleSim[3];
        double PosLAnklePitchSim[3];
        double PosLKneeSim[3];

        sensor_msgs::JointState jointsCtrl_, jointsSensor_;
        double cmd_l_vel, cmd_r_vel;

    private:

    };
}

#endif //webots
