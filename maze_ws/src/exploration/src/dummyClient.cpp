#include "ros/ros.h"
#include "exploration/Trigger.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummyClient");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<exploration::Trigger>("trigger");
  exploration::Trigger srv;

  while (ros::ok())
  {
    if (client.call(srv))
    {
      // ROS_INFO("Direction: %ld", (long int)srv.response.sum);
      ROS_INFO("Direction: %s", srv.response.message.c_str());
    }
    else
    {
      ROS_ERROR("Failed to call service Trigger");
    }    
  }
  
  return 0;
}