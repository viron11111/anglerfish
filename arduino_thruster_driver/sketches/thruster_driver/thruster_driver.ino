//#define TWI_FREQ 100000l // Can be changed to reduce I2C frequency

#include <Wire.h>
#include "Arduino_I2C_ESC.h"
#include <ros.h>
#include <std_msgs/Int16.h>

int speed1 = 0;
int speed2 = 0;
int speed3 = 0;
int speed4 = 0;
int speed5 = 0;
int speed6 = 0;
int speed7 = 0;
int speed8 = 0;

Arduino_I2C_ESC motor1(0x29);
Arduino_I2C_ESC motor2(0x2A);
Arduino_I2C_ESC motor3(0x2B);
Arduino_I2C_ESC motor4(0x2C);
Arduino_I2C_ESC motor5(0x2D);
Arduino_I2C_ESC motor6(0x2E);
Arduino_I2C_ESC motor7(0x2F);
Arduino_I2C_ESC motor8(0x30);

ros::NodeHandle  nh;

void thruster_string1( const std_msgs::Int16& thrust_cmd){
  speed1 = thrust_cmd.data;
}

void thruster_string2( const std_msgs::Int16& thrust_cmd){
  speed2 = thrust_cmd.data;
}

void thruster_string3( const std_msgs::Int16& thrust_cmd){
  speed3 = thrust_cmd.data;
}

void thruster_string4( const std_msgs::Int16& thrust_cmd){
  speed4 = thrust_cmd.data;
}

void thruster_string5( const std_msgs::Int16& thrust_cmd){
  speed5 = thrust_cmd.data;
}

void thruster_string6( const std_msgs::Int16& thrust_cmd){
  speed6 = thrust_cmd.data;
}

void thruster_string7( const std_msgs::Int16& thrust_cmd){
  speed7 = thrust_cmd.data;
}

void thruster_string8( const std_msgs::Int16& thrust_cmd){
  speed8 = thrust_cmd.data;
}

ros::Subscriber<std_msgs::Int16> sub1("thruster_cmd1", &thruster_string1 );
ros::Subscriber<std_msgs::Int16> sub2("thruster_cmd2", &thruster_string2 );
ros::Subscriber<std_msgs::Int16> sub3("thruster_cmd3", &thruster_string3 );
ros::Subscriber<std_msgs::Int16> sub4("thruster_cmd4", &thruster_string4 );
ros::Subscriber<std_msgs::Int16> sub5("thruster_cmd5", &thruster_string5 );
ros::Subscriber<std_msgs::Int16> sub6("thruster_cmd6", &thruster_string6 );
ros::Subscriber<std_msgs::Int16> sub7("thruster_cmd7", &thruster_string7 );
ros::Subscriber<std_msgs::Int16> sub8("thruster_cmd8", &thruster_string8 );

void setup() {
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(sub4);
  nh.subscribe(sub5);
  nh.subscribe(sub6);
  nh.subscribe(sub7);
  nh.subscribe(sub8);
  
  Wire.begin();

  // Optional: Add these two lines to slow I2C clock to 12.5kHz from 100 kHz
  // This is best for long wire lengths to minimize errors
  //TWBR = 158;  
  //TWSR |= bit (TWPS0);
}

void loop() {
  
  motor1.set(speed1);
  motor2.set(speed2);
  motor3.set(speed3);
  motor4.set(speed4);
  motor5.set(speed5);
  motor6.set(speed6);
  motor7.set(speed7);
  motor8.set(speed8);  
  motor1.update();
  motor2.update();
  motor3.update();
  motor4.update();
  motor5.update();
  motor6.update();
  motor7.update();
  motor8.update();  

  nh.spinOnce();
  //delay(250);

}
