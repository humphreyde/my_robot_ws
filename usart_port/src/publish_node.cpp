
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
//test send value
float test_send_joint[7]   = {0.000000, 0.000000, 0.00000, 0.000000, 0.000000, 0.000000,  0.000000 };
unsigned char testSend3=0x05;

//test receive value
float joint_pos_now[7]   = {0};

unsigned char testRece4=0x00;


/*********************************回调函数***********************************/
//关节控制回调函数
void leftArmController_Callback(const baxter_core_msgs::JointCommand &msg)
{
  for(int i=0;i<7;i++)
  {
    test_send_joint[i] = msg.command[i];
  }
}

void leftArmJointStatePub(ros::Publisher &pub, float joint_state[7])
{
  sensor_msgs::JointState msg;
    for(int i = 0; i<7;i++)
    {
      msg.position[i]= joint_state[i];
    }
    pub.publish(msg);

}

int main(int agrc,char **argv)
{
    ros::init(agrc,argv,"usart_send");
    ros::NodeHandle nh;

    ros::Subscriber leftarm_controller_sub = nh.subscribe("/robot/controller", 1000, &leftArmController_Callback);
    //w
    ros::Publisher 	robot_joint_state_pub = nh.advertise<sensor_msgs::JointState>("/robot/joint_states", 1000);

	ros::Rate loop_rate(10);
    
    //串口初始
    serialInit();

    while(ros::ok())
    {
      //调取回调函数一次，获取当前关节角度
      ros::spinOnce();
      //向STM32端发送数据，float类型，最后一个为unsigned char类型
	    writePosition(test_send_joint[0],test_send_joint[1],test_send_joint[2],test_send_joint[3],test_send_joint[4],test_send_joint[5],test_send_joint[6],testSend3);
      
      //从STM32接收数据，输入参数依次转化为（角度）、预留控制位
	    readPosition( joint_pos_now, &testRece4);
     //发布机械臂的当前角度
      leftArmJointStatePub(robot_joint_state_pub,joint_pos_now);
        //打印数据
	    ROS_INFO("%f,%f,%f,%f,%f,%f,%f\n",test_send_joint[0],test_send_joint[1],test_send_joint[2],test_send_joint[3],test_send_joint[4],test_send_joint[5],test_send_joint[6]);

      loop_rate.sleep();
    }
    return 0;
}
 



