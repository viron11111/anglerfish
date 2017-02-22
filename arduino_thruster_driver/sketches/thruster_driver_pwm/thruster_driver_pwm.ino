#define TWI_FREQ 12500l // Can be changed to reduce I2C frequency

//#include <Wire.h>
//#include "Arduino_I2C_ESC.h"
#include <ros.h>
#include <std_msgs/Int16.h>
#include <Servo.h>

int led = 13;
// Variables will change:
int ledState = LOW;             // ledState used to set the LED
long previousMillis = 0;        // will store last time LED was updated

// the follow variables is a long because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long interval = 100;           // interval at which to blink (milliseconds)

int speed1 = 1500;
int speed2 = 1500;
int speed3 = 1500;
int speed4 = 1500;
int speed5 = 1500;
int speed6 = 1500;
int speed7 = 1500;
int speed8 = 1500;

Servo thruster1;
Servo thruster2;
Servo thruster3;
Servo thruster4;
Servo thruster5;
Servo thruster6;
Servo thruster7;
Servo thruster8;

ros::NodeHandle  nh;

void thruster_string1( const std_msgs::Int16& thrust_cmd){
  speed1 = thrust_cmd.data;
  if (speed1 == 0){
    speed1 = 1500;
  }
  else if (speed1 >= 1900){
    speed1 = 1900;
  }
  else if (speed1 <= 1100){
    speed1 = 1100;
  }
}

void thruster_string2( const std_msgs::Int16& thrust_cmd){
  speed2 = thrust_cmd.data;
  if (speed2 == 0){
    speed2 = 1500;
  }
  else if (speed2 >= 1900){
    speed2 = 1900;
  }
  else if (speed2 <= 1100){
    speed2 = 1100;
  }
}

void thruster_string3( const std_msgs::Int16& thrust_cmd){
  speed3 = thrust_cmd.data;
  if (speed3 == 0){
    speed3 = 1500;
  }
  else if (speed3 >= 1900){
    speed3 = 1900;
  }
  else if (speed3 <= 1100){
    speed3 = 1100;
  }
}

void thruster_string4( const std_msgs::Int16& thrust_cmd){
  speed4 = thrust_cmd.data;
  if (speed4 == 0){
    speed4 = 1500;
  }
  else if (speed4 >= 1900){
    speed4 = 1900;
  }
  else if (speed4 <= 1100){
    speed4 = 1100;
  } 
}

void thruster_string5( const std_msgs::Int16& thrust_cmd){
  speed5 = thrust_cmd.data;
  if (speed5 == 0){
    speed5 = 1500;
  }
  else if (speed5 >= 1900){
    speed5 = 1900;
  }
  else if (speed5 <= 1100){
    speed5 = 1100;
  }  
}

void thruster_string6( const std_msgs::Int16& thrust_cmd){
  speed6 = thrust_cmd.data;
  if (speed6 == 0){
    speed6 = 1500;
  }
  else if (speed6 >= 1900){
    speed6 = 1900;
  }
  else if (speed6 <= 1100){
    speed6 = 1100;
  }
}

void thruster_string7( const std_msgs::Int16& thrust_cmd){
  speed7 = thrust_cmd.data;
  if (speed7 == 0){
    speed7 = 1500;
  }
  else if (speed7 >= 1900){
    speed7 = 1900;
  }
  else if (speed7 <= 1100){
    speed7 = 1100;
  } 
}

void thruster_string8( const std_msgs::Int16& thrust_cmd){
  speed8 = thrust_cmd.data;
  if (speed8 == 0){
    speed8 = 1500;
  }
  else if (speed8 >= 1900){
    speed8 = 1900;
  }
  else if (speed8 <= 1100){
    speed8 = 1100;
  }
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
  pinMode(led, OUTPUT);
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(sub4);
  nh.subscribe(sub5);
  nh.subscribe(sub6);
  nh.subscribe(sub7);
  nh.subscribe(sub8);
  
  thruster1.attach(3);
  thruster2.attach(4);
  thruster3.attach(5);
  thruster4.attach(6);
  thruster5.attach(9);
  thruster6.attach(10);
  thruster7.attach(23);
  thruster8.attach(22);

  thruster1.writeMicroseconds(1500); // send "stop" signal to ESC.
  thruster2.writeMicroseconds(1500); // send "stop" signal to ESC.
  thruster3.writeMicroseconds(1500); // send "stop" signal to ESC.
  thruster4.writeMicroseconds(1500); // send "stop" signal to ESC.
  thruster5.writeMicroseconds(1500); // send "stop" signal to ESC.
  thruster6.writeMicroseconds(1500); // send "stop" signal to ESC.
  thruster7.writeMicroseconds(1500); // send "stop" signal to ESC.
  thruster8.writeMicroseconds(1500); // send "stop" signal to ESC.
  delay(1000); // delay to allow the ESC to recognize the stopped signal
}

void loop() {
  
  thruster1.writeMicroseconds(speed1);
  thruster2.writeMicroseconds(speed2); 
  thruster3.writeMicroseconds(speed3);
  thruster4.writeMicroseconds(speed4);
  thruster5.writeMicroseconds(speed5);
  thruster6.writeMicroseconds(speed6);
  thruster7.writeMicroseconds(speed7);
  thruster8.writeMicroseconds(speed8);

  unsigned long currentMillis = millis();
 
  if(currentMillis - previousMillis > interval) {
    // save the last time you blinked the LED 
    previousMillis = currentMillis;   

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW)
      ledState = HIGH;
    else
      ledState = LOW;

    // set the LED with the ledState of the variable:
    digitalWrite(led, ledState);
  }

  nh.spinOnce();
  //delay(250);

}
