#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"
#include "webots_api/webots.h"
#include "webots_api/position_random.h"
#include <vector>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include <sstream>
#include <stdlib.h>
#include <time.h>
#include <camera_info_manager/camera_info_manager.h>

webots_api::WebotsApi webots_api_;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "webots", ros::init_options::AnonymousName);

  ros::NodeHandle n;

  ros::Publisher sim_time_pub_ = n.advertise<rosgraph_msgs::Clock>("/clock", 1);
  ros::Publisher left_extra_rgb_pub_ = n.advertise<sensor_msgs::Image>("/camera/LeftExtraRGB", 10);
  ros::Publisher camera_info_pub = n.advertise<sensor_msgs::CameraInfo>("/camera/LeftExtraRGB_info",10);
  // ros::Publisher left_extra_depth_pub_ = n.advertise<sensor_msgs::Image>("/camera/LeftExtraDepth", 10);
  // ros::Publisher right_extra_rgb_pub_ = n.advertise<sensor_msgs::Image>("/camera/RightExtraRGB", 10);
  // ros::Publisher right_extra_depth_pub_ = n.advertise<sensor_msgs::Image>("/camera/RightExtraDepth", 10);

  //simulation time
  rosgraph_msgs::Clock sim_time;
  ros::Time time_ros_pub_(0,0);
  sim_time.clock.nsec = 0;
  sim_time.clock.sec  = 0;
  ros::Duration one_ms(0, 1e6);

  ros::Rate loop_rate(1000);

  double time_sim = 0;

  webots_api_.CameraApiInit();

  //相机参数info
  sensor_msgs::CameraInfo msg;
  msg.height = 640;
  msg.width = 640;
  msg.distortion_model = "plumb_bob";
  msg.D.resize(5);//如果不resize，编译不报错，运行时会出现段错误 失真参数
  for (int i = 0; i<5; i++)
  {
      //畸变参数
      msg.D[i] = 0;
  }
  //旋转矩阵R，双目相机有效，使得左右极线平行
  float R_[] = {1.0, 0.0, 0.0, 
                0.0, 1.0, 0.0, 
                0.0, 0.0, 1.0};
  //相机内参K，fx，0，cx，0，fy，cy，0，0，1
  float K_[] = {521.1711084,  0.0,          320,
                0.0,          547.7089685,  240,
                0,            0,            1};
  //投影矩阵P，fx',0,cx',Tx,0,fy',cy',Ty,0,0,1,0,   在单目相机中，Tx=Ty=0
  float P_[] = {521.1711084,  0,            320,  0,
                0,            547.7089685,  240,  0,
                0,            0,            1,    0};
  for (int i = 0; i<9; i++)
  {
	  //9  旋转矩阵
      msg.R[i] = R_[i] ;
      //9 内参矩阵     fx 0  cx   0  fy  cy  0 0 1
      msg.K[i] = K_[i];
     
  }
  for (int i = 0; i<12; i++)
  {
      //12 投影矩阵 fx 0 cx tx 0 fy cy ty 0 0 1 0单目相机tx=ty=0
      msg.P[i] = P_[i];
  }
  msg.binning_x = 0;
  msg.binning_y = 0;
  msg.roi.x_offset = 0;
  msg.roi.y_offset = 0;
  msg.roi.height = 0;
  msg.roi.width = 0;
  msg.roi.do_rectify = false;

  //simulation two iterations to avoid the error feedback
  wb_robot_step(1);
  wb_robot_step(1);

  int count = 0;//publish camera 32hz
  while (ros::ok())
  {
    time_sim += 0.001;
   //控制器容易崩溃

    sim_time.clock += one_ms;
    time_ros_pub_ += one_ms;
    if(sim_time.clock.nsec>=1e9)
    {
       sim_time.clock.nsec -= 1e9;
       sim_time.clock.sec += 1;
    }
    //30Hz publish
    if(count > 30){

      //camera publish
      webots_api_.cameraLeftExtraRGB(webots_api_.imageLeftExtraRGB_);
      webots_api_.imageLeftExtraRGB_.header.stamp = time_ros_pub_;
      left_extra_rgb_pub_.publish(webots_api_.imageLeftExtraRGB_);
      camera_info_pub.publish(msg);
      // webots_api_.cameraLeftExtraDepth(webots_api_.imageLeftExtraDepth_);
      // webots_api_.imageLeftExtraDepth_.header.stamp = time_ros_pub_;
      // left_extra_depth_pub_.publish(webots_api_.imageLeftExtraDepth_);      

      // webots_api_.cameraRightExtraRGB(webots_api_.imageRightExtraRGB_);
      // webots_api_.imageRightExtraRGB_.header.stamp = time_ros_pub_;
      // right_extra_rgb_pub_.publish(webots_api_.imageRightExtraRGB_);

      // webots_api_.cameraRightExtraDepth(webots_api_.imageRightExtraDepth_);
      // webots_api_.imageRightExtraDepth_.header.stamp = time_ros_pub_;
      // right_extra_depth_pub_.publish(webots_api_.imageRightExtraDepth_);  
      count = 0;
    }else{
      count++;
    }
    wb_robot_step(1);
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}


// %EndTag(FULLTEXT)%
