#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "roomba_base_link");
  ros::NodeHandle n;

  ros::Rate r(2);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        //pitch, roll, and YAW (rotation along the vertical axis) in radians (i think ccw is positive)
        tf::Transform(tf::Quaternion(0, 0, 0, 1),
        //x, y, and z in meters
        tf::Vector3(0.1, 0.0, 0.2)),
        //measure those things from base link to base lazer being positive
        ros::Time::now(),"base_link", "scan"));
    r.sleep();
  }
}

