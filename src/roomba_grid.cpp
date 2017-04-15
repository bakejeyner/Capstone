#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int8MultiArray.h>

#include <vector>

//width height and resolution of the map
  int height;
  int width;
  double resolution;
  int heightCount;
  int widthCount;
  std::vector<signed char> roombaGrid;
  int middle;
  
  double prevx;
  double prevy;
  
void fillSpot(double odomx, double odomy)
{ 
  prevx = odomx;
  prevy = odomy;
  
  int threshold = 5;
  
  int newPoint = middle - ((int) odomy/resolution*widthCount) + ((int) odomx/resolution);
  int shiftedPoint;
  
  //don't check if we've been here
  if (roombaGrid[newPoint] == 2) return;
  
  ROS_INFO("newPoint: %d", newPoint);
  
  roombaGrid[newPoint] = 2;
  
  for (int i = -threshold; i <= threshold; i++) //vertical
  {
    for (int j = -threshold; j <= threshold; j++) //horizontal
    {
      shiftedPoint = newPoint - i*widthCount + j;
      ROS_INFO("shiftedPoint: %d", shiftedPoint);
      if (!(shiftedPoint > heightCount*widthCount-1) && !(shiftedPoint < 0))
      {
        if (roombaGrid[shiftedPoint] == 0) roombaGrid[shiftedPoint] = 1;
      }
    }
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "roomba_grid");
  
  //width height and resolution of the map
  height = 21;
  width = 21;
  resolution = .04;
  heightCount = height/resolution;
  widthCount = width/resolution;
  middle = heightCount*widthCount/2;
  roombaGrid.resize(widthCount*heightCount);
  
  fillSpot(0, 0);
  

  ros::NodeHandle n;
  tf::StampedTransform odomTransform;
  tf::TransformListener odomListener;
  ros::Publisher grid_publisher = n.advertise<std_msgs::Int8MultiArray>("roomba_grid", 5);
  
  ros::Rate r(1);
  while(n.ok())
  {
    std_msgs::Int8MultiArray msg;
    
    //look for odom
    try
    {
      odomListener.lookupTransform("map", "base_link", ros::Time(0), odomTransform);
      fillSpot(odomTransform.getOrigin().x(), odomTransform.getOrigin().y());
    }
    catch (tf::TransformException ex)
    {
      ROS_INFO("Exception in tf stuff");
    }
    
    msg.data = roombaGrid;
    grid_publisher.publish(msg);
    ROS_INFO("published grid");
    
    ros::spinOnce();
    r.sleep();
  }
  
}
