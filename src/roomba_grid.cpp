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
  
  double prevx;
  double prevy;
  
void fillSpot(double odomx, double odomy)
{
  //don't check if we havn't moved
  if (odomx == prevx && odomy == prevy) return;
  
  prevx = odomx;
  prevy = odomy;
  
  int threshold = 5;
  
  int newPoint = middle - ((int) odomy/resolution*widthCount) + ((int) odomx/resolution);
  int shiftedPoint;
  
  ROS_INFO("newPoint: %d", newPoint);
  
  for (int i = -threshold; i <= threshold; i++) //vertical
  {
    for (int j = -threshold; j <= threshold; j++) //horizontal
    {
      shiftedPoint = newPoint - i*widthCount + j;
      ROS_INFO("shiftedPoint: %d", shiftedPoint);
      if (!(shiftedPoint > heightCount*widthCount) && !(shiftedPoint < 0))
      {
        roombaGrid[shiftedPoint] = 1;
      }
    }
  }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  fillSpot(msg->pose.pose.position.x, msg->pose.pose.position.y);
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
  roombaGrid = (int*) calloc ((heightCount*widthCount), sizeof(int));
  
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
    ROS_INFO("published grid");
    
    ros::spinOnce();
    r.sleep();
  }
  
}
