#include <ros/ros.h>
#include <flappy_automation_code/FlappyController.hpp>


int main(int argc, char **argv)
{
  ros::init(argc,argv,"flappy_automation_code");
  auto handle_ptr = ros::NodeHandlePtr{new ros::NodeHandle};
  auto node = FlappyController{handle_ptr};
  ros::spin();
  return 0;
}
