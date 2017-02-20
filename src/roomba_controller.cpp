#include "ros/ros.h"
#include "roomba_tf/convert.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "roomba_controller");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<roomba_tf::convert>("/controller_conversion", true);
  
  roomba_tf::convert srv;
  
  ros::Rate r(5);
  
  ROS_INFO("In controller");
  
  while (n.ok())
  {
    ROS_INFO("In controller while loop");
  
    srv.request.start = true;
  
    if (client.call(srv))
    {
      ROS_INFO(srv.response.finish ? "Successfully called service convert: True" : "Successfully called service convert: False");
    }
    
    else
    {
      ROS_ERROR("Failed to call service convert");
      return 1;
    }
    
    r.sleep();
  }

  return 0;
}
