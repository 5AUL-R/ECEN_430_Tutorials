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

> Note: The following chapters are written for ROS Melodic, in most cases other versions of ROS can be used by substituting the version name when installing packages. i.e` $ sudo apt-get install ros-kinetic-urg-node`.


LiDAR or Light Detection and Ranging is a technique used for determining the range to objects. The LiDARs you have today are the URG-04LX-UG01 and cost around \$1500 NZD. If you where writing code for these LiDARs from scrath you would need to write a UART driver and pharser to recive and unpack the data. However, with ROS someone has done the grunt work for you.

The primary and perfered way of installing ROS nodes onto your systems is to use the `apt-get` command. Use the command below install the URG node onto your systems. 
```
 $ sudo apt-get install ros-melodic-urg-node 
 ```
 
Now plug in your LiDAR to the USB port on your NUC, the spindle should spin up and the LED start flashing. Use the`$ dmesg | grep tty` command to identify what serial port the LiDAR has been assigned. By default it should be on `/dev/ttyACM0`. The image below shows what the dmesg output should look like. 

IMAGE HERE

Using terminal, start the ROS Master:
```
$ roscore
```
Open a new terminal tab using `CTRL + SHIFT + T`. Using this terminal tab start the urg_node with:
```
    $ rosrun urg_node urg_node _serial_port:=/dev/ttyACM0
```
Where the serial port is set to the serial port of the LiDAR. We can now verify that the LiDAR is sending data using the rostopic command line tool. Fist check that the the urg\_node is publishing data on the `/scan`. In a new terminal tab, use the command:

```
    $ rostopic list
```

If the `scan` topic shows up in the list, use rosoptic to check the publish frequency of the topic with:

```
    $ rostopic hz /scan
```

If the LiDAR is running correctly it should be publishing data at 10~Hz. We can now visualize the data using rviz, launch it in a new terminal tab with:
```
    $ rviz
```

Using the panel on the left of RVIZ add the LiDAR data as shown below. To get the laser to show up, you will need change the fixed frame of rviz from "map" to "laser". This tells RVIZ to display all the data in relation to the coordinate system defined by the laser, not the map.  

IMAGE HERE

Have a play with the LiDAR and figure out it's maximum range and scan area. Once you are done stop all the running programs. 








 
 


