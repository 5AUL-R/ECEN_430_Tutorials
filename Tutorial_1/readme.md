# ECEN 430 Tutorial Part 1

## 1A - ROS and Ubuntu Introduction 

### Command Line Refresher
ROS uses the Linux command line for the majority of programming and execution tasks. 
Before starting with ROS ensure that you are up to speed on how to navigate the Linux command line efficiently. 
The below lists common commands that you will use with the command line. 



### ROS Introduction 
ROS or Robot Operating System is middle-ware that sits between high-level control and the sensor/actuators. ROS was initially developed at Stanford University but has become a worldwide standard for robotic development. The ROS system consists of nodes, each node has a compute function to gather, manipulate or display data. Data is transferred between nodes on topics. A node can subscribe or publish data to/from any topic. Each topic has a message type, a message defines how the data is packetised. At a low-level ROS uses the TCP/IP stack for transferring data, this allows easy expansion over networked systems.  

#### Nodes 
A node is a process that performs computation, A combination of nodes is used to form a system. Nodes will usually operate on a fine grain scale, with one node responsible solely for one part of the system. For example, one node controls a laser range-finder, one Node controls the robot's wheel motors, one node performs localisation, one node performs path planning, one node provides a graphical view of the system, and so on.

By implementing a robotic system with this level of modularity, the system is easily adaptable and fault-tolerant. More information on nodes can be found [here](http://wiki.ros.org/Nodes). 


#### Topics
A topic is a bus over which nodes exchange messages. Nodes can send/receive data from a topic by publishing or subscribing to it. Generally, a node will not know who is publishing the data it is subscribed to, nor will it know who it is publishing too. A topic can have multiple subscribers and publishers. 

Topics are intended for unidirectional transport of data between nodes, for more advanced bidirectional data, services are used. Services are out of the scope of this tutorial, but more info can be found [here]( http://wiki.ros.org/Services)

Topics use the TCP/IP network communication layer, this means that nodes can be spread across the series of networked processors for distributed computation. More information on topics [here](http://wiki.ros.org/Topics)  

#### Messages 
Messages are how data is packetized when it is sent over a topic. A topic can only ever transport one message type.  Each message can consist of basic data types (int, char, string), more advanced ROS specific types (Header, pose) or other messages.   



#### ROS Master (or ROS core)
When a node is initialised, it needs to know what other nodes are active in the system and where to send its data. This is the role of the ROS master. 

The ROS master is not actively involved in data transfer, rather, it makes sure all the nodes in the system are aware of each other. There must only be one ROS master in each ROS system. Typically the ROS master is wrapped in the ROS Core, a series of critical services and nodes for ROS.

More information on the [ROS master](http://wiki.ros.org/Master) and [ROS core](http://wiki.ros.org/roscore)   

## 1B - LiDAR


