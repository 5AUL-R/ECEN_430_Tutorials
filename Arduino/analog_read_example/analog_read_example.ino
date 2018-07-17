/* 
 * Pot Example for Rosserial
 */

#include <ros.h>
#include <ECEN_430_Tutorials/adc_msg.h>


ros::NodeHandle nh;
ECEN_430_Tutorials::adc_msg msg;
ros::Publisher adc_publisher("ADC_topic", &msg);

const int pot_pin = A0;
const int led_pin = 13;



void publishReading(){
 int adcReading = analogRead(pot_pin);
 float voltage = (adcReading/1023.0)*5.0; 
  msg.adc_reading = adcReading;
  msg.voltage = voltage;
  adc_publisher.publish(&msg);
  
}


void setup()
{
  nh.initNode();
  nh.advertise(adc_publisher);
 
}

void loop()
{
  

  publishReading();
  nh.spinOnce();
  delay(100);
}
