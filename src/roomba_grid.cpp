#include <ros/ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <nav_msgs/Odometry.h>

//width height and resolution of the map
  int height;
  int width;
  double resolution;
  int heightCount;
  int widthCount;
  int *roombaGrid;
  int middle;
  
void fillSpot(double odomx, double odomy)
{
  int threshold = 3;
  
  int newPoint = middle - odomy/resolution*widthCount + odomx/resolution;
  
  for (int i = -threshold; i < threshold; i++)
  {
    for (int j = -threshold; j < threshold; j++)
    {
      if (!(newPoint - i*widthCount + j > heightCount*widthCount) && !(newPoint - i*widthCount + j < 0))
      {
        roombaGrid[newPoint] = 1;
      }
    }
  }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  fillSpot(msg->pose.pose.position.x, msg->pose.pose.position.y);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "roomba_tf");
  
  //width height and resolution of the map
  height = 21;
  width = 21;
  resolution = .04;
  heightCount = height/resolution;
  widthCount = width/resolution;
  roombaGrid = (int*) calloc ((heightCount*widthCount), sizeof(int));
  
  
  //find the middle of the grid as our starting point
  middle = heightCount*widthCount/2;
  
  fillSpot(0, 0);
  

  ros::NodeHandle n;
  ros::Subscriber odom_listener = n.subscribe("odom", 5, odomCallback);
  ros::Publisher grid_publisher = n.advertise<std_msgs::Int8MultiArray>("roomba_grid", 5);
  
  ros::Rate r(1);
  while(n.ok())
  {
    std_msgs::Int8MultiArray msg;
    //vector<int> vRoombaGrid(roombaGrid, roombaGrid + sizeof roombaGrid / sizeof roombaGrid[0]);
    msg.data.insert(msg.data.end(), &roombaGrid[0], &roombaGrid[heightCount*widthCount]);
    grid_publisher.publish(msg);
    
    ros::spinOnce();
    r.sleep();
  }
  
}
