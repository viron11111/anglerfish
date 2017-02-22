#include <ros.h>
#include <ros/time.h>
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Imu.h"
#include <tf/tf.h>
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/TransformStamped.h"

#include <Wire.h>
#include "SFE_HMC6343.h"

ros::NodeHandle nh;

geometry_msgs::Vector3 magVec;
sensor_msgs::MagneticField mag_msg;
sensor_msgs::Imu imu_msg;

ros::Publisher pub_mag("/imu/mag_hmc6343", &mag_msg);
ros::Publisher pub_imu("/imu/imu_hmc6343", &imu_msg);

SFE_HMC6343 compass; // Declare the sensor object

int magx = 0;
int magy = 0;
int magz = 0;
bool led = 0;
float accelx = 0.0;
float accely = 0.0;
float accelz = 0.0;
float mag_cov[9] = {0.01,0,0,  //x
                    0,0.01,0,  //y
                    0,0,0.01}; //z

void setup()
{
  pinMode(13, OUTPUT);
  // Start serial communication at 115200 baud
  Serial.begin(115200);
  
  // Give the HMC6343 a half second to wake up
  delay(500); 
  
  // Initialize the HMC6343 and verify its physical presence
  if (!compass.init())
  {
    Serial.println("Sensor Initialization Failed\n\r"); // Report failure, is the sensor wiring correct?
  }

  // Set sample rate to 10 Hz
  compass.writeEEPROM(OP_MODE2, 0b00000010);
  delay(10);

  nh.initNode();
  nh.advertise(pub_mag);
  nh.advertise(pub_imu);
}

void loop()
{
  // Read, calculate, and print the heading, pitch, and roll from the sensor
  compass.readHeading();
  //printHeadingData();
  
  // Read, calculate, and print the acceleration on the x, y, and z axis of the sensor
  compass.readAccel();
  compass.readMag();

  float Yaw =   (((float) compass.heading/10.0));//*3.14159)/180.0;
  float Pitch = (((float) compass.pitch  /10.0));//*3.14159)/180.0;
  float Roll =  (((float) compass.roll   /10.0));//*3.14159)/180.0;

  if (Yaw > 6000.0){
    Yaw = Yaw - 6553.6;
  }
  else{
    Yaw = Yaw;
  }
  if (Roll > 6000.0){
    Roll = Roll - 6553.6;
  }
  else{
    Roll = Roll;
  }
  if (Pitch > 6000.0){
    Pitch = Pitch - 6553.6;
  }
  else{
    Pitch = Pitch;
  }

  Roll = (Roll*3.14159)/180.0;
  Pitch = (Pitch*3.14159)/180.0;
  Yaw = (Yaw*3.14159)/180.0;

  Serial.print("RPY,");
  Serial.print(Roll,5);
  Serial.print(",");
  Serial.print(Pitch,5);
  Serial.print(",");
  Serial.println(Yaw,5);

  float c1 = cos(Yaw/2);
  float c2 = cos(Pitch/2);
  float c3 = cos(Roll/2);
  float s1 = sin(Yaw/2);
  float s2 = sin(Pitch/2);
  float s3 = sin(Roll/2);
  float w = c1*c2*c3 - s1*s2*s3;
  float x = s1*s2*c3 + c1*c2*s3;
  float y = s1*c2*c3 + c1*s2*s3;
  float z = c1*s2*c3 - s1*c2*s3;

  /*Serial.print(w);Serial.print(",");
  Serial.print(x);Serial.print(",");
  Serial.print(y);Serial.print(",");
  Serial.println(z);*/

  if (compass.magX > 60000){
    magx = compass.magX - 65536;
  }
  else{
    magx = compass.magX;
  }
  if (compass.magY > 60000){
    magy = compass.magY - 65536;
  }
  else{
    magy = compass.magY;
  }
  if (compass.magZ > 60000){
    magz = compass.magZ - 65536;
  }
  else{
    magz = compass.magZ;
  }

  if (compass.accelX > 60000){
    accelx = compass.accelX - 65536;
  }
  else{
    accelx = compass.accelX;
  }
  if (compass.accelY > 60000){
    accely = compass.accelY - 65536;
  }
  else{
    accely = compass.accelY;
  }
  if (compass.accelZ > 60000){
    accelz = compass.accelZ - 65536;
  }
  else{
    accelz = compass.accelZ;
  }

  accelx = (float) accelx/1024.0;
  accely = (float) accely/1024.0;
  accelz = (float) accelz/1024.0;

  mag_msg.header.frame_id = "/hmc6343";
  mag_msg.header.stamp = nh.now();
  mag_msg.magnetic_field.x = (float)magx/1000.0;
  mag_msg.magnetic_field.y = (float)magy/1000.0;
  mag_msg.magnetic_field.z = (float)magz/1000.0;
  //mag_msg.magnetic_field_covariance = mag_cov;
  pub_mag.publish(&mag_msg);

  imu_msg.header.frame_id = "/hmc6343";
  imu_msg.header.stamp = nh.now();
  imu_msg.linear_acceleration.x = accelx;
  imu_msg.linear_acceleration.y = accely;
  imu_msg.linear_acceleration.z = accelz;
  /*imu_msg.orientation.w = w;
  imu_msg.orientation.x = x;
  imu_msg.orientation.y = y;
  imu_msg.orientation.z = z;*/
  pub_imu.publish(&imu_msg);  
  
  nh.spinOnce();
  
  if (led == 0)
  {
    led = 1;
  }
  else
  {
    led = 0;
  }
  digitalWrite(13, led);
  delay(100); // Minimum delay of 100ms (HMC6343 has 10Hz sensor reads/calculations)
}

void printHeadingData()
{
  Serial.println("Heading Data (Raw value, in degrees):");
  Serial.print("Heading: ");
  Serial.print(compass.heading); Serial.print("  "); // Print raw heading value
  Serial.print((float) compass.heading/10.0);Serial.write(176);Serial.println(); // Print heading in degrees
  Serial.print("Pitch: ");
  Serial.print(compass.pitch); Serial.print("  ");
  Serial.print((float) compass.pitch/10.0);Serial.write(176);Serial.println();
  Serial.print("Roll: ");
  Serial.print(compass.roll); Serial.print("  ");
  Serial.print((float) compass.roll/10.0);Serial.write(176);Serial.println();
  Serial.println();
}

// Print both the raw values of the compass acceleration measured on each axis
// as well as calculate and print the accelerations in g forces
// Sample Output:
// Accelerometer Data (Raw value, in g forces):
// X: -52    -0.05g
// Y: -44    -0.04g
// Z: -1047  -1.02g
void printAccelData()
{
  Serial.println("Accelerometer Data (Raw value, in g forces):");
  Serial.print("X: ");
  Serial.print(compass.accelX); Serial.print("  "); // Print raw acceleration measurement on x axis
  Serial.print((float) compass.accelX/1024.0);Serial.println("g"); // Print x axis acceleration measurement in g forces
  Serial.print("Y: ");
  Serial.print(compass.accelY); Serial.print("  ");
  Serial.print((float) compass.accelY/1024.0);Serial.println("g");
  Serial.print("Z: ");
  Serial.print(compass.accelZ); Serial.print("  ");
  Serial.print((float) compass.accelZ/1024.0);Serial.println("g");
  Serial.println();
}
