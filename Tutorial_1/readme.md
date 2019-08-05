# ECEN 430 Tutorial Part 1

## ROS and Ubuntu Introduction 

### Command Line Refresher
ROS uses the Linux command line for the majority of programming and execution tasks. 
Before starting with ROS ensure that you are up to speed on how to navigate the Linux command line efficiently. 
The below lists common commands that you will use with the command line. 



### ROS Introduction 
ROS or Robot Operating System is middle-ware that sits between high-level control and the sensor/actuators. ROS was initially developed at Stanford University, but has become a worldwide standard for robotic development. The ROS system consists of nodes, each node has a compute function to gather, manipulate or display data. Data is transferred between nodes on topics. A node can subscribe or publish data to/from any topic. Each topic has a message type, a message defines how the data is packetised. At a low level ROS uses the TCP/IP stack for transferring data, this allows easy expansion over networked systems.  
