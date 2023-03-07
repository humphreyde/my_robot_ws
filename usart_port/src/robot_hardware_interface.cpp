#include "usart_port/robot_hardware_interface.h"

boost::mutex Arm_io_mutex;

ArmHWInterface::ArmHWInterface()
{
    for(int i=0; i<7; i++)
    {
        std::string joint_cmd_name="arm_joint_";
        std::string joint_state_name="arm_joint_";
        std::string joint_num=boost::lexical_cast<std::string>(i+1);
        joint_cmd_name.append(joint_num);
        joint_state_name.append(joint_num);
        joint_cmd_name.append("_controller/command");
        joint_state_name.append("_controller/state");
        // pub_cmd[i]=_n.advertise<std_msgs::Float64>(joint_cmd_name, 1);
        // sub_js[i]=_n.subscribe(joint_state_name, 1, &ArmHWInterface::read, this);
        client_controller_list=_n.serviceClient<controller_manager_msgs::ListControllers>("/controller_manager/list_controllers");
        loaded_flag=false;
    }

    for(int i=0; i<7; i++)
    {
        std::string joint_name="arm_joint_";
        std::string joint_num=boost::lexical_cast<std::string>(i+1);
        joint_name.append(joint_num);
        hardware_interface::JointStateHandle jnt_state_handle_tmp(joint_name, &pos[i], &vel[i], &eff[i]);
        jnt_state_interface.registerHandle(jnt_state_handle_tmp);
    }

    registerInterface(&jnt_state_interface);

    for(int i=0; i<7; i++)
    {
        std::string joint_name="arm_joint_";
        std::string joint_num=boost::lexical_cast<std::string>(i+1);
        joint_name.append(joint_num);
        hardware_interface::JointHandle jnt_handle_tmp(jnt_state_interface.getHandle(joint_name), &cmd[i]);
        jnt_cmd_interface.registerHandle(jnt_handle_tmp);
    }

    registerInterface(&jnt_cmd_interface);

    start_time_=ros::Time::now();
    start_dur_.operator +=(ros::Duration(3));
}

void ArmHWInterface::publishCmd()
{
    if(ros::Time::now()-start_time_<start_dur_)
        return;
    // for(int i=0; i<4; i++)
    // {
    //     cmd_msg[i].data=cmd[i];
    //     pub_cmd[i].publish(cmd_msg[i]);
    // }
}

void ArmHWInterface::read()
{
    
}

ros::NodeHandle ArmHWInterface::getnode()
{
    return _n;
}

static void timespecInc(struct timespec &tick, int nsec)
{
  int SEC_2_NSEC = 1e+9;
  tick.tv_nsec += nsec;
  while (tick.tv_nsec >= SEC_2_NSEC)
  {
    tick.tv_nsec -= SEC_2_NSEC;
    ++tick.tv_sec;
  }
}

void* update_loop(void* threadarg)
{
    controller_manager::ControllerManager *c=(controller_manager::ControllerManager *)threadarg;
    ros::Rate r(50);
    ros::Duration d(0.02);

    while(ros::ok())
    {
        boost::mutex::scoped_lock lock(Arm_io_mutex);
        c->update(ros::Time::now(), d);
        lock.unlock();
        r.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"Robot_hardware_interface", ros::init_options::AnonymousName);
    ArmHWInterface c1;


    controller_manager::ControllerManager cm(&c1);
    pthread_t tid;
    pthread_create(&tid, NULL, update_loop, (void *)&cm);


    ROS_INFO("Robot bring up successfully");
    // loop
    ros::Rate r(50);
    while(ros::ok())
    {
        boost::mutex::scoped_lock lock(Arm_io_mutex);
        c1.publishCmd();
        lock.unlock();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}

