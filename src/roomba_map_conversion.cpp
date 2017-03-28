#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

nav_msgs::OccupancyGrid

int main(int argc, char** argv){
  ros::init(argc, argv, "roomba_map_conversion");
  
  ros::Subscriber costmapGridSub = n.subscribe("costmap_2d", 3, costmapGridCallback);
  ros::Publisher mapPub = n.advertise("map", 5);
  
  ros::rate r(2);
  
  while (n.ok())
  {
    
    r.sleep()
  }
  
}
