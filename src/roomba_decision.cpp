#include "ros/ros.h"
#include <std_msgs/Int8MultiArray.h>
#include <nav_msgs/OccupancyGrid.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

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

const signed char *roombaGrid;
const signed char *costmapGrid;


void roombaGridCallback(const std_msgs::Int8MultiArray::ConstPtr& msg)
{
  roombaGrid = &(msg->data[0]);
}

void costmapGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  costmapGrid = &(msg->data[0]);
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
      if ((roombaGrid[currentIndex] == 0) && (costmapGrid[currentIndex] < costmapThreshold))
      {
        lastLoopValue = i;
        return currentIndex;
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
      if ((roombaGrid[currentIndex] == 0) && (costmapGrid[currentIndex] < costmapThreshold))
      {
        lastLoopValue = i;
        return currentIndex;
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
      if ((roombaGrid[currentIndex] == 0) && (costmapGrid[currentIndex] < costmapThreshold))
      {
        lastLoopValue = i;
        return currentIndex;
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
      if ((roombaGrid[currentIndex] == 0) && (costmapGrid[currentIndex] < costmapThreshold))
      {
        lastLoopValue = i;
        return currentIndex;
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

    ROS_INFO("Salman: Initializing width height and resolution of the map (after rosinit)");
    fflush(stdout);
      
  //node shit
  ros::NodeHandle n;
  
  //make a roomba grid and costmap grid subscriber
  ros::Subscriber roombaGridSub = n.subscribe("roomba_grid", 3, roombaGridCallback);
  ros::Subscriber costmapGridSub = n.subscribe("costmap_2d", 3, costmapGridCallback);
  
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
  
    ROS_INFO("Salman: Initializing width height and resolution of the map");
    fflush(stdout);
  
  //make a message variable
  move_base_msgs::MoveBaseGoal goal;
  
  distance = 1;
  lastLoop = 1;
  lastLoopValue = 99999;
  
  int currentIndex;
  
      ROS_INFO("Salman: before node while loop");
      fflush(stdout);
  while ((n.ok()) && (distance <= heightCount/2))
  {
    ros::spinOnce();
          ROS_INFO("Salman: before fourForLoops");
          fflush(stdout);
    currentIndex = fourForLoops();
          ROS_INFO("Salman: after fourForLoops :)");
          fflush(stdout);
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
        ROS_INFO("yaay!!");
      else
        ROS_INFO("naay!!");
    }
  }
  
  return 0;
}
