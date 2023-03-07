

#ifndef ROBOT_HW_INTERFACE
#define ROBOT_HW_INTERFACE

#include <ros/ros.h>
#include <urdf/model.h>
#include <pthread.h>
#include <time.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include "baxter_core_msgs/JointCommand.h"

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <controller_manager/controller_manager.h>
#include <controller_manager_msgs/ListControllers.h>

class ArmHWInterface : public hardware_interface::RobotHW
{
public:
    ArmHWInterface();
    void read();
    void publishCmd();
    ros::NodeHandle getnode();

private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_cmd_interface;
    double cmd[7];
    double pos[7];
    double vel[7];
    double eff[7];

    controller_manager_msgs::ListControllersRequest list_req;
    controller_manager_msgs::ListControllersResponse list_resp;

    bool loaded_flag;

    ros::NodeHandle _n;
    ros::Publisher pub_cmd;//发布关节命令
    std_msgs::Float64 cmd_msg[7];
    ros::Time start_time_;
    ros::Duration start_dur_;

    ros::Subscriber sub_js;//订阅关节状态
    ros::ServiceClient client_controller_list;

};

#endif

