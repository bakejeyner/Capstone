#include "ros/ros.h"
#include <std_msgs/Int8MultiArray.h>
#include <nav_msgs/OccupancyGrid.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int height;
int width;
double resolution;
int heightCount;
int widthCount;
int middle;
int distance;
int lastLoop;
int lastLoopValue;
double x; 
double y;

const int costmapThreshold = 50;

std::vector<signed char> roombaGrid;
std::vector<signed char> costmapGrid;


void roombaGridCallback(const std_msgs::Int8MultiArray::ConstPtr& msg)
{
  ROS_INFO("Read Roomba Grid");
  roombaGrid = msg->data;
  
  ROS_INFO("before test");
  roombaGrid[141494];
  ROS_INFO("after test");
  
  ROS_INFO("after Read Roomba Grid");
}

void costmapGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  ROS_INFO("Read Costmap Grid");
  costmapGrid = msg->data;
  
  ROS_INFO("before test");
  costmapGrid[141494];
  ROS_INFO("after test");
  
  
  ROS_INFO("after Read Costmap Grid");
}

int fourForLoops()
{
  int currentIndex;
  if(lastLoop <= 1)
  {
    //check to see if we've been in this loop before
    if (lastLoopValue == 99999) lastLoopValue = -distance;
    
    for (int i = lastLoopValue; i < distance; i++)
    {
      currentIndex = middle - i*widthCount + distance;
      ROS_INFO("currentIndex: %d", currentIndex);
      ROS_INFO("In loop: 1");
      if (!(currentIndex > heightCount*widthCount-1) && !(currentIndex < 0))
      {
        ROS_INFO("before array accesses");
        if ((roombaGrid[currentIndex] == 0) && (costmapGrid[currentIndex] < costmapThreshold))
        {
          ROS_INFO("inside array accesses");
          lastLoopValue = i;
          return currentIndex;
        }
        ROS_INFO("after array accesses");
      }
    }
    
    lastLoopValue = 99999;
    lastLoop++;
  }
  
  if(lastLoop <= 2)
  {
    //check to see if we've been in this loop before
    if (lastLoopValue == 99999) lastLoopValue = -distance;
    
    for (int i = lastLoopValue; i < distance; i++)
    {
      currentIndex = middle - distance*widthCount - i;
      ROS_INFO("currentIndex: %d", currentIndex);
      ROS_INFO("In loop: 2");
      if (!(currentIndex > heightCount*widthCount-1) && !(currentIndex < 0))
      {
        ROS_INFO("before array accesses");
        if ((roombaGrid[currentIndex] == 0) && (costmapGrid[currentIndex] < costmapThreshold))
        {
          ROS_INFO("inside array accesses");
          lastLoopValue = i;
          return currentIndex;
        }
        ROS_INFO("after array accesses");
      }
    }
    
    lastLoopValue = 99999;
    lastLoop++;
  }
  
  if(lastLoop <= 3)
  {
    //check to see if we've been in this loop before
    if (lastLoopValue == 99999) lastLoopValue = -distance;
    
    for (int i = lastLoopValue; i < distance; i++)
    {
      currentIndex = middle + i*widthCount - distance;
      ROS_INFO("currentIndex: %d", currentIndex);
      ROS_INFO("In loop: 3");
      if (!(currentIndex > heightCount*widthCount-1) && !(currentIndex < 0))
      {
        ROS_INFO("before array accesses");
        if ((roombaGrid[currentIndex] == 0) && (costmapGrid[currentIndex] < costmapThreshold))
        {
          ROS_INFO("inside array accesses");
          lastLoopValue = i;
          return currentIndex;
        }
        ROS_INFO("after array accesses");
      }
    }
    
    lastLoopValue = 99999;
    lastLoop++;
  }
  
  if(lastLoop <= 4)
  {
    //check to see if we've been in this loop before
    if (lastLoopValue == 99999) lastLoopValue = -distance;
    
    for (int i = lastLoopValue; i < distance; i++)
    {
      currentIndex = middle + distance*widthCount + i;
      ROS_INFO("currentIndex: %d", currentIndex);
      ROS_INFO("In loop: 4");
      if (!(currentIndex > heightCount*widthCount-1) && !(currentIndex < 0))
      {
        ROS_INFO("before array accesses");
        if ((roombaGrid[currentIndex] == 0) && (costmapGrid[currentIndex] < costmapThreshold))
        {
          ROS_INFO("inside array accesses");
          lastLoopValue = i;
          return currentIndex;
        }
        ROS_INFO("after array accesses");
      }
    }
    
    lastLoopValue = 99999;
    lastLoop = 1;
  }
  return -1;
}

//main waits for convert service
int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "roomba_decision");
      
  //node shit
  ros::NodeHandle n;
  
  //make a roomba grid and costmap grid subscriber
  ros::Subscriber roombaGridSub = n.subscribe("roomba_grid", 5, roombaGridCallback);
  ros::Subscriber costmapGridSub = n.subscribe("move_base/global_costmap/costmap", 5, costmapGridCallback);
  
  //make service and wait
  MoveBaseClient ac("move_base", true);
  while(!ac.waitForServer(ros::Duration(5))) {}
  
  //width height and resolution of the map
  height = 21;
  width = 21;
  resolution = .04;
  heightCount = height/resolution;
  widthCount = width/resolution;
  middle = heightCount*widthCount/2;
  
  //make a message variable
  move_base_msgs::MoveBaseGoal goal;
  
  distance = 1;
  lastLoop = 1;
  lastLoopValue = 99999;
  
  int currentIndex;
  
  ros::Rate r(1);

  while ((n.ok()) && (distance <= heightCount/2))
  {
    ROS_INFO("before spin");
    ros::spinOnce();
    ROS_INFO("after spin");
    
    if (roombaGrid.size() == 0 || costmapGrid.size() == 0)
    {
      ROS_INFO("waiting for roombagrid and costmapgrid");
      r.sleep();
      continue;
    }
    
    ROS_INFO("before four loops");
    currentIndex = fourForLoops();
    ROS_INFO("after four loops");

    //if the for loops didn't find anything
    if (currentIndex == -1) distance += 2;
    
    //else the for loops found something call the service
    else
    {
      goal.target_pose.header.frame_id = "base_link";
      goal.target_pose.header.stamp = ros::Time::now();
     
     /* 
      int x = currentIndex;
      int y = 0;
      while (x > widthCount)
      {
        x -= widthCount;
        y++;
      }
      */
      
      x = (currentIndex % widthCount - widthCount/2)*resolution;
      y = (-currentIndex / widthCount + widthCount/2)*resolution;
     // x = 10;
     // y = 5;
      
      ROS_INFO("Current Index: %d", currentIndex);
      ROS_INFO("X Value: %f", x);
      ROS_INFO("Y Value: %f", y);
      
      goal.target_pose.pose.position.x = x;
      goal.target_pose.pose.position.y = y;
      
      switch(lastLoop)
      {
        case 1:
          goal.target_pose.pose.orientation.x = 0.0;
          goal.target_pose.pose.orientation.y = 0.0;
          goal.target_pose.pose.orientation.z = .7071;
          goal.target_pose.pose.orientation.w = .7071;
          break;
          
        case 2:
          goal.target_pose.pose.orientation.x = 0.0;
          goal.target_pose.pose.orientation.y = 0.0;
          goal.target_pose.pose.orientation.z = 1.0;
          goal.target_pose.pose.orientation.w = 0.0;
          break;
          
        case 3:
          goal.target_pose.pose.orientation.x = 0.0;
          goal.target_pose.pose.orientation.y = 0.0;
          goal.target_pose.pose.orientation.z = -.7071;
          goal.target_pose.pose.orientation.w = .7071;
          break;
          
        case 4:
          goal.target_pose.pose.orientation.x = 0.0;
          goal.target_pose.pose.orientation.y = 0.0;
          goal.target_pose.pose.orientation.z = 0.0;
          goal.target_pose.pose.orientation.w = 1.0;
          break;
      }
      
      ac.sendGoal(goal);
      ac.waitForResult();
      
      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Nav Goal Success!");
      else
        ROS_INFO("Nav Goal Failure");
    }
  }
  
  return 0;
}
