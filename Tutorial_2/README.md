# Part 2 – Custom Nodes and other things 
## Part 2.1 – Custom Nodes
Today we are going to write a custom node that will take in a LiDAR scan on the `\scan` topic and invert it (like the LiDAR is mounted upside down). We’ll start by subscribing to the data, then publish the inverted data. Before we start writing code, we need to create a new ROS package in the `catkin_ws/src` directory. Use the `catkin_create_pkg`, you’ll need to the include the `roscpp, rospy, std_msgs and sensor_msgs` dependencies. Name the package something sensible for the function it will have, ROS package names should be all lower case and written in snake case, not camel case. The docs for `catkin_create_pkg` are [here](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_create.html)     
## 2.1A – Subscribing 
Move into the package you just created and create and CD into a new `/src` directory. Now create a new C++ file call `scan_inverter.cpp` and open it with gedit. Copy and paste the boilerplate code from the `ros_node_boiler_plate.cpp` file in this repo, into your new C++ file.       












