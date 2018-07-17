# ECEN_430_Tutorials
For this Tutorial to work you need to clone this repo to `~/catkin_ws/src/`:

```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/skipper762/ECEN_430_Tutorials
```

Now we need to compile the code
```
$ cd ~/catkin_ws/
$ catkin_make
```
this should return no errors.


## Part 1 - Blink
Today we are extending ROS nodes over serial to Arduinos. This is done with `rosserial`. To install this use:

```
$ sudo apt-get install ros-kinetic-rosserial-arduino
$ sudo apt-get install ros-kinetic-rosserial
```

Initially we are using a simple blink sketch on the Arduino, this is found the `Arduino` folder of this repo. Before this sketch will compile we need to generate the ROS headers. Move to the sketch book, and make the header files:

```
$ cd ~/sketchbook/libraries/
$ rosrun rosserial_arduino make_libraries.py .
```

Compile and upload this sketch to the Arduino. Once upload has completed start rosserial with the correct port.

```
$ rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0
```
Remember you need to start the ROS Master, you may also need to change the serial port. Now use `rostopic` to publish a blank message to the Arduino:

```
$ rostopic pub toggle_led std_msgs/Empty --once
```
You should see the LED blink every time you send a message.

Make sure you fully understand how this is working before continuing to part two. 

## Part 2 - Analog Read 
We are going to use a custom message to communicate over serial from the Arduino. The message definition can be found in: `/msg/adc_msg.msg`. I have modified the package.xml and CMakeLists.txt for this package so the messages will be generated. [Tutorial Here](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_msg)

Connect the POT as a voltage divider to Analog Pin 0 of the Arduino.  
Upload the anaload_read_example sketch to the Arduino.

Start rosserial on the correct port
Use `rostopic` to verify that the correct data is being sent from the Arduino.

Open the `src/adc_listener.cpp` file in this direcotry. We are now going to run this cpp file. 
I have already modified package.xml and CMakeLists.txt so the code is compiled. Have a look at these before continuing. 

To run the code we must first tell the terminal where the executables are stored. 

```
$ $ source ~/catkin_ws/devel/setup.bash
```

We can then run the code with:
```
$ rosrun ECEN_430_Tutorials adc_listener 
```

Use RQT graph to see how this publisher subscriber combination works. 

# Part 3 - IMU

In your boxes you can find the sparkfun razor IMUs, using these, make a sketch the reports the IMU data back to the computer over rosserial. 

[Product Page](https://www.sparkfun.com/products/14001)
[Getting Started Guide](https://learn.sparkfun.com/tutorials/9dof-razor-imu-m0-hookup-guide?_ga=2.188812459.1607136359.1531795723-2044257458.1531795723) 


I expect that this will take you more than the remaining time.








