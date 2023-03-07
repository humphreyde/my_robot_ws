#include "webots_api/webots.h"

/**
 * @namespace webots_api
 */
namespace webots_api{
    /**
     * @class WebotsApi
     */

    //constructor
    WebotsApi::WebotsApi() {}

    //destructor
    WebotsApi::~WebotsApi() {}

    // //api
    //arm read
    bool WebotsApi::ArmJointApiRead(sensor_msgs::JointState &msgs_out, const int mode) {

        //left
        msgs_out.name[LShoulderPitch] = "joint1";
        msgs_out.position[LShoulderPitch] = wb_position_sensor_get_value(LShoulderPitchSensor);
        msgs_out.velocity[LShoulderPitch] = wb_motor_get_velocity(LShoulderPitchMotor);
        msgs_out.effort[LShoulderPitch] = wb_motor_get_torque_feedback(LShoulderPitchMotor);

        msgs_out.name[LShoulderRoll] = "joint2";
        msgs_out.position[LShoulderRoll] = wb_position_sensor_get_value(LShoulderRollSensor);
        msgs_out.velocity[LShoulderRoll] = wb_motor_get_velocity(LShoulderRollMotor);
        msgs_out.effort[LShoulderRoll] = wb_motor_get_torque_feedback(LShoulderRollMotor);

        msgs_out.name[LShoulderYaw] = "joint3";
        msgs_out.position[LShoulderYaw] = wb_position_sensor_get_value(LShoulderYawSensor);
        msgs_out.velocity[LShoulderYaw] = wb_motor_get_velocity(LShoulderYawMotor);
        msgs_out.effort[LShoulderYaw] = wb_motor_get_torque_feedback(LShoulderYawMotor);

        msgs_out.name[LElbowRoll] = "joint4";
        msgs_out.position[LElbowRoll] = wb_position_sensor_get_value(LElbowRollSensor);
        msgs_out.velocity[LElbowRoll] = wb_motor_get_velocity(LElbowRollMotor);
        msgs_out.effort[LElbowRoll] = wb_motor_get_torque_feedback(LElbowRollMotor);

        msgs_out.name[LElbowYaw] = "joint5";
        msgs_out.position[LElbowYaw] = wb_position_sensor_get_value(LElbowYawSensor);
        msgs_out.velocity[LElbowYaw] = wb_motor_get_velocity(LElbowYawMotor);
        msgs_out.effort[LElbowYaw] = wb_motor_get_torque_feedback(LElbowYawMotor);

        msgs_out.name[LWristPitch] = "joint6";
        msgs_out.position[LWristPitch] = wb_position_sensor_get_value(LWristPitchSensor);
        msgs_out.velocity[LWristPitch] = wb_motor_get_velocity(LWristPitchMotor);
        msgs_out.effort[LWristPitch] = wb_motor_get_torque_feedback(LWristPitchMotor);

        msgs_out.name[LWristRoll] = "joint7";
        msgs_out.position[LWristRoll] = wb_position_sensor_get_value(LWristRollSensor);
        msgs_out.velocity[LWristRoll] = wb_motor_get_velocity(LWristRollMotor);
        msgs_out.effort[LWristRoll] = wb_motor_get_torque_feedback(LWristRollMotor);

        if(mode != 6 && mode != 7){
          double dMaxTorque[MOTOR_NUMBERS],dMinTorque[MOTOR_NUMBERS];
          dMaxTorque[LShoulderPitch] = wb_motor_get_max_torque(LShoulderPitchMotor );
          dMaxTorque[LShoulderRoll ] = wb_motor_get_max_torque(LShoulderRollMotor  );
          dMaxTorque[LShoulderYaw  ] = wb_motor_get_max_torque(LShoulderYawMotor   );
          dMaxTorque[LElbowRoll    ] = wb_motor_get_max_torque(LElbowRollMotor     );
          dMaxTorque[LElbowYaw     ] = wb_motor_get_max_torque(LElbowYawMotor      );
          dMaxTorque[LWristPitch   ] = wb_motor_get_max_torque(LWristPitchMotor    );
          dMaxTorque[LWristRoll    ] = wb_motor_get_max_torque(LWristRollMotor     );

          for(int i = LShoulderPitch; i <= LWristRoll; i++){
            dMinTorque[i] = - dMaxTorque[i];
          }

          for(int i = LShoulderPitch; i <= LWristRoll; i++){
            msgs_out.velocity[i] = 1*DesiredVelocity[i][0];
//            msgs_out.effort[i] = 1*DesiredTorque[i][0];
            msgs_out.effort[i] = std::min(dMaxTorque[i],std::max(dMinTorque[i], DesiredTorque[i][0]));
          }
        }

       return true;
    }

    //ApiWrite

    bool WebotsApi::LeftArmJointApiWrite(const sensor_msgs::JointState msgs_in, const int mode){

      if(6 == mode){
        wb_motor_set_position(LShoulderPitchMotor,INFINITY);
        wb_motor_set_velocity(LShoulderPitchMotor,msgs_in.position[LShoulderPitch]);
        wb_motor_set_position(LShoulderRollMotor,INFINITY);
        wb_motor_set_velocity(LShoulderRollMotor,msgs_in.position[LShoulderRoll]);
        wb_motor_set_position(LShoulderYawMotor,INFINITY);
        wb_motor_set_velocity(LShoulderYawMotor,msgs_in.position[LShoulderYaw]);
        wb_motor_set_position(LElbowRollMotor,INFINITY);
        wb_motor_set_velocity(LElbowRollMotor,msgs_in.position[LElbowRoll]);
        wb_motor_set_position(LElbowYawMotor,INFINITY);
        wb_motor_set_velocity(LElbowYawMotor,msgs_in.position[LElbowYaw]);
        wb_motor_set_position(LWristPitchMotor,INFINITY);
        wb_motor_set_velocity(LWristPitchMotor,msgs_in.position[LWristPitch]);
        wb_motor_set_position(LWristRollMotor,INFINITY);
        wb_motor_set_velocity(LWristRollMotor,msgs_in.position[LWristRoll]);
      }else if(7 == mode){
        wb_motor_set_torque(LShoulderPitchMotor,msgs_in.position[LShoulderPitch]);
        wb_motor_set_torque(LShoulderRollMotor,msgs_in.position[LShoulderRoll]);
        wb_motor_set_torque(LShoulderYawMotor,msgs_in.position[LShoulderYaw]);
        wb_motor_set_torque(LElbowRollMotor,msgs_in.position[LElbowRoll]);
        wb_motor_set_torque(LElbowYawMotor,msgs_in.position[LElbowYaw]);        
        wb_motor_set_torque(LWristPitchMotor,msgs_in.position[LWristPitch]);
        wb_motor_set_torque(LWristRollMotor,msgs_in.position[LWristRoll]);
      }else{//5 of init

#if 0
        for(int i = LShoulderPitch; i <= LWristRoll; i++){
          DesiredPosition[i][1]=DesiredPosition[i][0];
          DesiredPosition[i][0]=msgs_in.position[i];
          DesiredVelocity[i][0]=(DesiredPosition[i][0]-DesiredPosition[i][1])/0.001;
          FeedbackPosition[i][1] =FeedbackPosition[i][0];
        }

          FeedbackPosition[LShoulderPitch][0] = wb_position_sensor_get_value(LShoulderPitchSensor);
          FeedbackPosition[LShoulderRoll ][0] = wb_position_sensor_get_value(LShoulderRollSensor);
          FeedbackPosition[LShoulderYaw  ][0] = wb_position_sensor_get_value(LShoulderYawSensor);
          FeedbackPosition[LElbowRoll    ][0] = wb_position_sensor_get_value(LElbowRollSensor);
          FeedbackPosition[LElbowYaw     ][0] = wb_position_sensor_get_value(LElbowYawSensor);          
          FeedbackPosition[LWristPitch   ][0] = wb_position_sensor_get_value(LWristPitchSensor);
          FeedbackPosition[LWristRoll    ][0] = wb_position_sensor_get_value(LWristRollSensor);

          for(int i = LShoulderPitch; i <= LWristRoll; i++){
            FeedbackVelocity[i][0] = (FeedbackPosition[i][0] - FeedbackPosition[i][1])/0.001;
          }

          //wb_motor_set_torque
          double dMaxTorque[MOTOR_NUMBERS],dMinTorque[MOTOR_NUMBERS];
          dMaxTorque[LShoulderPitch] = wb_motor_get_max_torque(LShoulderPitchMotor );
          dMaxTorque[LShoulderRoll ] = wb_motor_get_max_torque(LShoulderRollMotor  );
          dMaxTorque[LShoulderYaw  ] = wb_motor_get_max_torque(LShoulderYawMotor   );
          dMaxTorque[LElbowRoll    ] = wb_motor_get_max_torque(LElbowRollMotor     );
          dMaxTorque[LElbowYaw     ] = wb_motor_get_max_torque(LElbowYawMotor      );
          dMaxTorque[LWristPitch   ] = wb_motor_get_max_torque(LWristPitchMotor    );
          dMaxTorque[LWristRoll    ] = wb_motor_get_max_torque(LWristRollMotor     );

          for(int i = LShoulderPitch; i <= LWristRoll; i++){
            dMinTorque[i] = - dMaxTorque[i];
          }

          JointPID[LShoulderPitch][0] = 2800;
          JointPID[LShoulderRoll ][0] = 4700;
          JointPID[LShoulderYaw  ][0] = 4500;
          JointPID[LElbowRoll    ][0] = 4700;
          JointPID[LElbowYaw     ][0] = 2900;
          JointPID[LWristPitch   ][0] = 1400;
          JointPID[LWristRoll    ][0] = 1400;

          JointPID[LShoulderPitch][2] = 65;
          JointPID[LShoulderRoll ][2] = 80;
          JointPID[LShoulderYaw  ][2] = 40;
          JointPID[LElbowRoll    ][2] = 45;
          JointPID[LElbowYaw     ][2] = 15;
          JointPID[LWristPitch   ][2] = 14;
          JointPID[LWristRoll    ][2] = 14;

          for(int i = LShoulderPitch; i <= LWristRoll; i++){
            DesiredTorque[i][0]=JointPID[i][0]*(DesiredPosition[i][0]-FeedbackPosition[i][0])+  JointPID[i][2]*(DesiredVelocity[i][0]-FeedbackVelocity[i][0]);
          }

          wb_motor_set_torque(LShoulderPitchMotor, std::min(dMaxTorque[LShoulderPitch ],std::max(dMinTorque[LShoulderPitch ], DesiredTorque[LShoulderPitch ][0])));
          wb_motor_set_torque(LShoulderRollMotor,  std::min(dMaxTorque[LShoulderRoll  ],std::max(dMinTorque[LShoulderRoll  ], DesiredTorque[LShoulderRoll  ][0])));
          wb_motor_set_torque(LShoulderYawMotor,   std::min(dMaxTorque[LShoulderYaw   ],std::max(dMinTorque[LShoulderYaw   ], DesiredTorque[LShoulderYaw   ][0])));
          wb_motor_set_torque(LElbowRollMotor,     std::min(dMaxTorque[LElbowRoll     ],std::max(dMinTorque[LElbowRoll     ], DesiredTorque[LElbowRoll     ][0])));
          wb_motor_set_torque(LElbowYawMotor,      std::min(dMaxTorque[LElbowYaw      ],std::max(dMinTorque[LElbowYaw      ], DesiredTorque[LElbowYaw      ][0])));
          wb_motor_set_torque(LWristPitchMotor,    std::min(dMaxTorque[LWristPitch    ],std::max(dMinTorque[LWristPitch    ], DesiredTorque[LWristPitch    ][0])));
          wb_motor_set_torque(LWristRollMotor,     std::min(dMaxTorque[LWristRoll     ],std::max(dMinTorque[LWristRoll     ], DesiredTorque[LWristRoll     ][0])));
#endif

#if 1
        wb_motor_set_position(LShoulderPitchMotor,msgs_in.position[LShoulderPitch]);
        wb_motor_set_position(LShoulderRollMotor,msgs_in.position[LShoulderRoll]);
        wb_motor_set_position(LShoulderYawMotor,msgs_in.position[LShoulderYaw]);
        wb_motor_set_position(LElbowRollMotor,msgs_in.position[LElbowRoll]);
        wb_motor_set_position(LElbowYawMotor,msgs_in.position[LElbowYaw]);        
        wb_motor_set_position(LWristPitchMotor,msgs_in.position[LWristPitch]);
        wb_motor_set_position(LWristRollMotor,msgs_in.position[LWristRoll]);
#endif
      }

      return true;
    }
  


    //world frame
    bool WebotsApi::RobotPosition(geometry_msgs::Point &msg_out){

      double *pos = (double*)wb_supervisor_field_get_sf_vec3f(trans_field);
      msg_out.x = pos[2];
      msg_out.y = pos[0];
      msg_out.z = pos[1];

      return true;
    }

    //robot pose
    bool WebotsApi::PoseOfRobotRead(geometry_msgs::PoseWithCovarianceStamped &msg_out){

      double *pos = (double*)wb_supervisor_field_get_sf_vec3f(trans_field);
      msg_out.pose.pose.position.x = pos[0];
      msg_out.pose.pose.position.y = pos[1];
      msg_out.pose.pose.position.z = pos[2];

      double *angle = (double*)wb_supervisor_field_get_sf_rotation(pose_field);
      msg_out.pose.pose.orientation.w = cos(angle[3]/2);
      msg_out.pose.pose.orientation.x = angle[0]*sin(angle[3]/2);
      msg_out.pose.pose.orientation.y = angle[1]*sin(angle[3]/2);
      msg_out.pose.pose.orientation.z = angle[2]*sin(angle[3]/2);

      return true;
    }

    //set position of robot
    bool WebotsApi::SetPositionOfRobot(const double x, const double y, const double z,
                        const double rx, const double ry, const double rz, const double theta){
      double posValue[3];
      posValue[0] = x;
      posValue[1] = y;
      posValue[2] = z;
      wb_supervisor_field_set_sf_vec3f(trans_field, posValue);

      double rotValue[4];
      rotValue[0] = rx;
      rotValue[1] = ry;
      rotValue[2] = rz;
      rotValue[3] = theta;
      wb_supervisor_field_set_sf_rotation(pose_field, rotValue);

      return true;
    }

    //ApiInit
    bool WebotsApi::ApiInit() {

        wb_robot_init();

        time_step=1;

        // ---------------------------------get  motor-------------------------------------


        //left arms
        LShoulderPitchMotor = wb_robot_get_device("joint1");
        wb_motor_enable_torque_feedback(LShoulderPitchMotor, time_step);
        LShoulderRollMotor = wb_robot_get_device("joint2");
        wb_motor_enable_torque_feedback(LShoulderRollMotor, time_step);
        LShoulderYawMotor = wb_robot_get_device("joint3");
        wb_motor_enable_torque_feedback(LShoulderYawMotor, time_step);
        LElbowRollMotor = wb_robot_get_device("joint4");
        wb_motor_enable_torque_feedback(LElbowRollMotor, time_step);
        LElbowYawMotor = wb_robot_get_device("joint5");
        wb_motor_enable_torque_feedback(LElbowYawMotor, time_step);
        LWristPitchMotor = wb_robot_get_device("joint6");
        wb_motor_enable_torque_feedback(LWristPitchMotor, time_step);
        LWristRollMotor = wb_robot_get_device("joint7");
        wb_motor_enable_torque_feedback(LWristRollMotor, time_step);

        //// ---------------------------------get  position sensor-------------------------------------


        //left arm
        LShoulderPitchSensor = wb_robot_get_device("joint1_sensor");
        LShoulderRollSensor = wb_robot_get_device("joint2_sensor");
        LShoulderYawSensor = wb_robot_get_device("joint3_sensor");
        LElbowRollSensor = wb_robot_get_device("joint4_sensor");
        LElbowYawSensor = wb_robot_get_device("joint5_sensor");
        LWristPitchSensor = wb_robot_get_device("joint6_sensor");
        LWristRollSensor = wb_robot_get_device("joint7_sensor");


        //// ---------------------------------enable  position sensor-------------------------------------




        //left arm
        wb_position_sensor_enable(LShoulderPitchSensor, time_step);
        wb_position_sensor_enable(LShoulderRollSensor, time_step);
        wb_position_sensor_enable(LShoulderYawSensor, time_step);
        wb_position_sensor_enable(LElbowRollSensor, time_step);
        wb_position_sensor_enable(LElbowYawSensor, time_step);
        wb_position_sensor_enable(LWristPitchSensor, time_step);
        wb_position_sensor_enable(LWristRollSensor, time_step);



        // // ---------------------------------enable torque feedback for all joints------------------------------------- 




        // //left arm
        wb_motor_enable_torque_feedback(LShoulderPitchMotor, time_step);
        wb_motor_enable_torque_feedback(LShoulderRollMotor, time_step);
        wb_motor_enable_torque_feedback(LShoulderYawMotor, time_step);
        wb_motor_enable_torque_feedback(LElbowRollMotor, time_step);
        wb_motor_enable_torque_feedback(LElbowYawMotor, time_step);
        wb_motor_enable_torque_feedback(LWristPitchMotor, time_step);
        wb_motor_enable_torque_feedback(LWristRollMotor, time_step);


        jointsCtrl_.name.resize(MOTOR_NUMBERS);
        jointsCtrl_.position.resize(MOTOR_NUMBERS);
        jointsCtrl_.velocity.resize(MOTOR_NUMBERS);
        jointsCtrl_.effort.resize(MOTOR_NUMBERS);
        //resize uowu
        jointsSensor_.name.resize(MOTOR_NUMBERS);
        jointsSensor_.position.resize(MOTOR_NUMBERS);
        jointsSensor_.velocity.resize(MOTOR_NUMBERS);
        jointsSensor_.effort.resize(MOTOR_NUMBERS);
        cmd_l_vel = 0;
        cmd_r_vel = 0;

        return true;
    }
    bool WebotsApi::cameraLeftExtraRGB(sensor_msgs::Image &msgs_out) {
      const unsigned char *imageRGB = wb_camera_get_image(Left_F_Eye);
      msgs_out.width = wb_camera_get_width(Left_F_Eye);
      msgs_out.height = wb_camera_get_height(Left_F_Eye);
      msgs_out.encoding = sensor_msgs::image_encodings::BGRA8;
      msgs_out.step = sizeof(char) * 4 * msgs_out.width;
      msgs_out.data.resize(4 * msgs_out.height * msgs_out.width);
      memcpy(&msgs_out.data[0], imageRGB,
            sizeof(char) * 4 * msgs_out.height * msgs_out.width);
      return 1;
    }
    bool WebotsApi::CameraApiInit() {
      wb_robot_init();
      time_step=1;

      Left_F_Eye = wb_robot_get_device("camera");

      wb_camera_enable(Left_F_Eye, 33);
      return true;
    
    }

}//namespace gait
