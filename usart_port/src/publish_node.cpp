
#include "ros/ros.h"
#include "std_msgs/String.h" //use data struct of std_msgs/String  
#include "linux_serial.h"
#include <sensor_msgs/JointState.h>
#include "baxter_core_msgs/JointCommand.h"

/*
*该文件实现的主要功能为：
*接收来自stm32的当前关节角度值
*调用串口writePosition()，将接收到的来自ros节点的关节角度值发送给stm32
*/
using namespace std;
//test send value
float test_send_joint[7]   = {0.100000, 0.200000, 0.00000, 0.000000, 0.000000, 0.000000,  0.000000 };
unsigned char testSend3=0x01;

//test receive value
float joint_pos_now[7]   = {0};

unsigned char testRece4=0x01;
bool joint_cmd_got = false;  //当订阅到 关节命令 时为true


/*********************************回调函数***********************************/
//关节控制回调函数
//赋值关节控制角度
void leftArmController_Callback(const baxter_core_msgs::JointCommand &msg)
{
  cout <<"joint cmd call back..."<<endl;
  for(int i=0;i<7;i++)
  {
    test_send_joint[i] = msg.command[i];
  }
  
 writePosition(test_send_joint[0],test_send_joint[1],test_send_joint[2],test_send_joint[3],test_send_joint[4],test_send_joint[5],test_send_joint[6],testSend3);
 // ROS_INFO("pulish cmd:%f,%f,%f,%f,%f,%f,%f\n",test_send_joint[0],test_send_joint[1],test_send_joint[2],test_send_joint[3],test_send_joint[4],test_send_joint[5],test_send_joint[6]);
  
  joint_cmd_got = true;
}
//发布关节角度状态
void leftArmJointStatePub(ros::Publisher &pub, float *joint_state)
{
    string nameId = "1234567";
    string nameSeed ="joint";
    sensor_msgs::JointState msg;
    msg.name.resize(7);//必须加上resize
    msg.position.resize(7);
    msg.velocity.resize(7);
    msg.effort.resize(7);

    for(int i = 0; i<7;i++)
    {
      msg.name[i] = nameSeed+nameId[i];
      msg.position[i] = joint_state[i];
      msg.velocity[i] = 0;
      msg.effort[i] = 0;
      //msg.position[i]= joint_state[i];
    }

    pub.publish(msg);
}

int main(int agrc,char **argv)
{
    ros::init(agrc,argv,"usart_send");
    ros::NodeHandle nh;
    //订阅关节角命令
    ros::Subscriber leftarm_controller_sub = nh.subscribe("/robot/controller", 100, &leftArmController_Callback);
    //发布关节角状态
    ros::Publisher 	robot_joint_state_pub = nh.advertise<sensor_msgs::JointState>("/robot/joint_states", 100);
    ros::Duration(1.0).sleep(); //等待话题订阅

	  ros::Rate loop_rate(5);
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();

    //串口初始
    serialInit();

    while(ros::ok())
    {

      
      //向STM32端发送数据，float类型，最后一个为unsigned char类型
      // for(int i = 0;i<10;i++)
      // {
      //   test_send_joint[i]+=1;
      //   writePosition(test_send_joint[0],test_send_joint[1],test_send_joint[2],test_send_joint[3],test_send_joint[4],test_send_joint[5],test_send_joint[6],testSend3);
      //   ROS_INFO("pulish joint value:%f,%f,%f,%f,%f,%f,%f\n",test_send_joint[0],test_send_joint[1],test_send_joint[2],test_send_joint[3],test_send_joint[4],test_send_joint[5],test_send_joint[6]);
      //   loop_rate.sleep();
      // }
      
      //从STM32接收关节角度状态，输入参数依次转化为（角度）、预留控制位
	    readPosition( joint_pos_now, &testRece4);
      //转化为弧度制
      joint_pos_now[0]=-joint_pos_now[0];
      joint_pos_now[1]=-joint_pos_now[1];//加上偏置和正负

      joint_pos_now[3]=-joint_pos_now[3];
      joint_pos_now[4]=-joint_pos_now[4];
      joint_pos_now[5]=-joint_pos_now[5];
      joint_pos_now[6]=-joint_pos_now[6];

     //发布机械臂的当前角度
      ROS_INFO("receive joint_state:%f,%f,%f,%f,%f,%f,%f\n",joint_pos_now[0],joint_pos_now[1],joint_pos_now[2],joint_pos_now[3],joint_pos_now[4],joint_pos_now[5],joint_pos_now[6]);
      leftArmJointStatePub(robot_joint_state_pub,joint_pos_now);


      //调取回调函数一次，获取当前关节角度命令
      // 测试是否订阅关节角
      // while(!joint_cmd_got)
      // {
      //     ros::spinOnce();
      //     ros::Duration(0.1).sleep();
      //     cout<<"等待关节命令..."<<endl;
      // }
      ros::spinOnce();
      //向STM32端发送关节角度命令，float类型，最后一个为unsigned char类型
   
	    //loop_rate.sleep();

      
    }
    return 0;
}
 



