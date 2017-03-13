Execution generally follows this path:

// get the roomba ready for gmapping and communication to it
roslaunch roomba_tf roomba_movement.launch

// start gmapping to make a map
roslaunch roomba_tf roomba_gmapping.launch

//run this in the roomba_tf directory to save the map created by gmapping
rosrun map_server map_saver

//run this to start navigating
roslaunch roomba_tf roomba_navigation
