#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"
#include "webots_api/webots.h"
#include "ubt_core_msgs/JointCommand.h"
#include "baxter_core_msgs/JointCommand.h"
#include "webots_api/position_random.h"
#include <vector>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include <sstream>
#include <stdlib.h>
#include <time.h>

webots_api::WebotsApi webots_api_;
int left_arm_mode = 0;

void LeftArmJointDesiredSub(const baxter_core_msgs::JointCommand &msgIn){
  for (int i = 0; i < 7; i++) {
   webots_api_.jointsCtrl_.position[i+webots_api::LShoulderPitch] = msgIn.command[i];

    left_arm_mode = msgIn.mode;
    if(i == 0){
      std::cout << "mode " << left_arm_mode << " pos " << webots_api_.jointsCtrl_.position[i+webots_api::LShoulderPitch] << std::endl;
    }
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "webots", ros::init_options::AnonymousName);

  ros::NodeHandle n;

  ros::Publisher sim_time_pub_ = n.advertise<rosgraph_msgs::Clock>("/clock", 1);

  ros::Publisher larm_measured_joint_pub_ = n.advertise<sensor_msgs::JointState>("/robot/joint_states", 10);

  ros::Subscriber left_arm_desired_joint_sub_ = n.subscribe("/robot/controller", 10, &LeftArmJointDesiredSub);

  sensor_msgs::JointState larm_measured_joint_, rarm_measured_joint_, lhand_measured_joint_,
                          rhand_measured_joint_, head_measured_joint_;
  larm_measured_joint_.name.resize(7);
  larm_measured_joint_.position.resize(7);
  larm_measured_joint_.velocity.resize(7);
  larm_measured_joint_.effort.resize(7);


  geometry_msgs::Point robot_position_;
  geometry_msgs::PoseWithCovarianceStamped walker_pose_;

  //simulation time
  rosgraph_msgs::Clock sim_time;
  ros::Time time_ros_pub_(0,0);
  sim_time.clock.nsec = 0;
  sim_time.clock.sec  = 0;
  ros::Duration one_ms(0, 1e6);

  ros::Rate loop_rate(1000);

  double time_sim = 0;

  webots_api_.ApiInit();
  
   
    
  //simulation two iterations to avoid the error feedback
  wb_robot_step(1);
  wb_robot_step(1);

  int count = 0;//publish camera 32hz
  while (ros::ok())
  {
    time_sim += 0.001;
   //控制器容易崩溃

    webots_api_.ArmJointApiRead(webots_api_.jointsSensor_, left_arm_mode);

 
    for(int i = 0; i < 7; i++){
      larm_measured_joint_.position[i] = webots_api_.jointsSensor_.position[i+webots_api::LShoulderPitch];

      larm_measured_joint_.velocity[i] = webots_api_.jointsSensor_.velocity[i+webots_api::LShoulderPitch];

      larm_measured_joint_.effort[i] = webots_api_.jointsSensor_.effort[i+webots_api::LShoulderPitch];

    }


  //  left_arm_mode = 7;
  //  webots_api_.jointsCtrl_.position[webots_api::LShoulderPitch] = 10*sin(3.14*2*time_sim/1.0);
  //  std::cout << "time: " << time_sim << std::endl;


    webots_api_.LeftArmJointApiWrite(webots_api_.jointsCtrl_, left_arm_mode);



    sim_time.clock += one_ms;
    time_ros_pub_ += one_ms;
    if(sim_time.clock.nsec>=1e9)
    {
       sim_time.clock.nsec -= 1e9;
       sim_time.clock.sec += 1;
    }

    //30Hz publish
    if(count > 30){



      count = 0;
    }else{
      count++;
    }

    //publish

    larm_measured_joint_.header.stamp = time_ros_pub_;

    sim_time_pub_.publish(sim_time);
    larm_measured_joint_pub_.publish(larm_measured_joint_);


    wb_robot_step(1);


    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}


// %EndTag(FULLTEXT)%
