#include <ros.h>
#include <std_srvs/SetBool.h>

ros::NodeHandle  nh;

//ros::ServiceClient<'rov_kill', SetBool >;
ros::ServiceClient<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse> client("rov_kill");

std_srvs::SetBoolRequest kill;
std_srvs::SetBoolResponse status2;

void setup() {    
  nh.initNode(); 
  nh.serviceClient(client);           
  pinMode(11, INPUT);
  pinMode(12, INPUT);
}

void loop()                     
{
  if (digitalRead(11) == HIGH) { 
    kill.data = true; 
    client.call(kill, status2);
  }
  if (digitalRead(12) == HIGH) {  
    kill.data = false;  
    client.call(kill, status2);
  }
  nh.spinOnce();
}

