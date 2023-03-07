#include "ros/ros.h"
#include "walker_ik.h"
#include "rosgraph_msgs/Clock.h"
#include "ubt_core_msgs/JointCommand.h"
#include "baxter_core_msgs/JointCommand.h"
#include "std_msgs/String.h"
#include <sstream>

#include "webots_api/position_random.h"

using namespace walkerArm_IK;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test", ros::init_options::AnonymousName);

  ros::NodeHandle n;

  ros::Publisher test_pub_ = n.advertise<baxter_core_msgs::JointCommand>("/walker/head/controller", 10);

  baxter_core_msgs::JointCommand test_data_;
  test_data_.names.resize(20);
  test_data_.command.resize(20);

  ros::Rate loop_rate(1000);

//  ros::Time time_begin = ros::Time::now();

  double time = 0.0;

  webots_api::PositionRandom pos_random_;


  while (ros::ok())
  {

    time += 0.001;

    int ll = 0;
    test_data_.mode = 5;
    for(ll = 0; ll < 7; ll++){
      test_data_.command[1*ll] = 0.5*sin(time*2*3.14/1.0);
//      test_data_.command[2*ll+1] = 0.5*sin(time*2*3.14/1.0);
    }

    std::cout << "time_now " << time << std::endl;
    std::cout <<"position random   " << pos_random_.positionRandom(-0.3, 0.3) << std::endl;

    test_pub_.publish(test_data_);


    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}


// %EndTag(FULLTEXT)%
