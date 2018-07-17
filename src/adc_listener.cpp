#include <ros/ros.h>
#include <ECEN_430_Tutorials/adc_msg.h>


ros::NodeHandle nh;

void adc_cb(const ECEN_430_Tutorials::adc_msg incoming_msg){
	ROS_INFO("ADC READING: %d", incoming_msg.adc_reading);
	ROS_INFO("VOLTAGE: %f", incoming_msg.voltage);
}


int main(int argc, char **argv){
	ros::init(argc,argv, "adc_listner");
	ros::NodeHandle nh;
	ROS_INFO("Node Started");
	ros::Subscriber adc_sub = nh.subscribe("ADC_topic", 1000, adc_cb);
	ros::spin();

}



