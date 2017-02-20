#include "ros/ros.h"

#include "std_msgs/Int8.h"
#include "geometry_msgs/Twist.h"

#include "roomba_tf/convert.h"

#include <stdlib.h>
#include <math.h>

class Cmd_Conversion
{
  public:
    ros::NodeHandle *n;
    ros::Subscriber sub_cmd_vel;
    ros::Publisher *pub_cmd_roomba;
    bool done;
    void velCallback(const geometry_msgs::Twist::ConstPtr&);
    bool convertCallback(roomba_tf::convert::Request&, roomba_tf::convert::Response&);
};

void Cmd_Conversion::velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  //debug message
  ROS_INFO("\nlinear x: %f\nlinear y: %f\nlinear z: %f\nangular x: %f\nangular y: %f\nangular z: %f\n\n", msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);

  double angularThreshold = 0.1;
  double linearThreshold = 0.1;
  
  double vyaw = msg->angular.z;
  double vx = msg->linear.x;
  double vy = msg->angular.y;
  double v = sqrt(vx*vx + vy*vy);
  
  //debug variables
  ROS_INFO("\nvyaw: %f\nvx: %f\nvy: %f\nv: %f\n\n", vyaw, vx, vy, v);
   
  //messages
  std_msgs::Int8 msg_roomba;
  
  //handle rotation first
  if ( abs(vyaw) >= angularThreshold )
  {
  
    //ccw
    if (vyaw > 0)
    {
      msg_roomba.data = 2;
      pub_cmd_roomba->publish(msg_roomba);
      return;
    }
    
    
    //cw
    else
    {
      msg_roomba.data = 3;
      pub_cmd_roomba->publish(msg_roomba);
      return;
    }
  }
  
  
  //handle velocity if no rotation
  else if ( abs(v) >= linearThreshold )
  {
    ROS_INFO("Inside velocity publish thingy");
    msg_roomba.data = 1;
    pub_cmd_roomba->publish(msg_roomba);
    return;
  }
  
  
  //if no rotation or velocity set done flag
  else
  {
    done = true;
    return;
  }
}


bool Cmd_Conversion::convertCallback(roomba_tf::convert::Request& req, roomba_tf::convert::Response& res)
{
  done = false;
  
  ros::Rate r(2);
  
  sub_cmd_vel = n->subscribe("/cmd_vel", 100, &Cmd_Conversion::velCallback, this);
  
  ROS_INFO("In convertCallback");
  ROS_INFO(done ? "done true" : "done false");
  
  while( n->ok() )
  {
    ROS_INFO("In convertCallback while loop");
    
    ros::spinOnce();
    
    //if finished, exit
    if (done)
    {
      sub_cmd_vel.shutdown();
      res.finish = true;
      return true;
    }
    
    r.sleep();
  }
  
  return false;
}


//main waits for convert service
int main(int argc, char **argv)
{ 
  ROS_INFO("In cmd_conversion");
  ros::init(argc, argv, "roomba_cmd_conversion");

  //node shit
  ros::NodeHandle n;
  
  //Vel_conversion object
  Cmd_Conversion cmd;
  cmd.n = &n;
  
  ros::Publisher pub_cmd_roomba = n.advertise<std_msgs::Int8>("cmd_roomba", 10);
  cmd.pub_cmd_roomba = &pub_cmd_roomba;
  
  ros::ServiceServer service = n.advertiseService("/controller_conversion", &Cmd_Conversion::convertCallback, &cmd);

  //pump callbacks in a loop
  ros::spin();

  return 0;
}
