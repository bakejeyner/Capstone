#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "roomba_temp_map");
      
  //node shit
  ros::NodeHandle n;
  
  ros::Publisher grid_publisher = n.advertise<nav_msgs::OccupancyGrid>("map", 5);
  
  ros::Duration oneSec(1.0);
  
  nav_msgs::OccupancyGrid temp_map;
  
  temp_map.data = {0,0,0,0,0,0,0}
  
  for (int i = 0; i < 5; i++)
  {
    
  }
}
