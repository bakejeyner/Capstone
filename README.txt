Execution generally follows this path:

// get the roomba ready for gmapping and communication to it
roslaunch roomba_tf roomba_movement.launch

// run this to start gmapping and navigating
roslaunch roomba_tf roomba_navigation.launch

//
// at this point you can set a nav goal in rviz
//

// if you want to test our algorithms
roslaunch roomba_tf roomba_autonomy.launch


Other stuff
// run ca_driver on its own
rosrun ca_driver ca_driver base_frame:=base_link dev:=/dev/ttyUSB0 robot_model:=ROOMBA_400

// sample cmd_vel (linear x is forward, angular z is either cw or ccw)
rostopic pub -r 5 /cmd_vel geometry_msgs/Twist '{linear: {x: .1, y: 0, z: 0}, angular: {x: 0 y: 0, z: 0}}'

// sample vacuum tester message
rostopic pub -r 5 /vacuum_on std_msgs/Empty
