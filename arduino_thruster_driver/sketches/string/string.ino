const char EOPmarker = '.'; //This is the end of packet marker
char serialbuf[32]; //This gives the incoming serial some room. Change it if you want a longer incoming.
 
#include <SoftwareSerial.h>
#include <string.h> // we'll need this for subString
#define MAX_STRING_LEN 20 // like 3 lines above, change as needed.
 
#include <Wire.h>
#include "Arduino_I2C_ESC.h"

int thruster = 0;
int speed = 0;
int speed1 = 0;
int speed2 = 0;
int speed3 = 0;
int speed4 = 0;
int speed5 = 0;
int speed6 = 0;

Arduino_I2C_ESC motor1(0x29);
Arduino_I2C_ESC motor2(0x2A);
Arduino_I2C_ESC motor3(0x2B);
Arduino_I2C_ESC motor4(0x2C);
Arduino_I2C_ESC motor5(0x2D);
Arduino_I2C_ESC motor6(0x2E);
 
void setup(){
  Serial.begin(57600);
  Wire.begin();
  TWBR = 158;  
  TWSR |= bit (TWPS0);
}
 
void loop() {
    if (Serial.available() > 0) { //makes sure something is ready to be read
      static int bufpos = 0; //starts the buffer back at the first position in the incoming serial.read
      char inchar = Serial.read(); //assigns one byte (as serial.read()'s only input one byte at a time
      if (inchar == 'k'){
          speed1 = 0;
          speed2 = 0;
          speed3 = 0;
          speed4 = 0;
          speed5 = 0;
          speed6 = 0;
        }
      else{
        if (inchar != EOPmarker) { //if the incoming character is not the byte that is the incoming package ender
          serialbuf[bufpos] = inchar; //the buffer position in the array get assigned to the current read
          bufpos++; //once that has happend the buffer advances, doing this over and over again until the end of package marker is read.
        }
        else { //once the end of package marker has been read
          serialbuf[bufpos] = 0; //restart the buff
          bufpos = 0; //restart the position of the buff
    
          int thruster = atoi(subStr(serialbuf, ",", 1));
          int speed = atoi(subStr(serialbuf, ",", 2));
    
          switch (thruster){
            case(1):
              speed1 = speed;
              break;
            case(2):
              speed2 = speed;
              break;
            case(3):
              speed3 = speed;
              break;
            case(4):
              speed4 = speed;
              break;      
            case(5):
              speed5 = speed;
              break;
            case(6):
              speed6 = speed;
              break;
          }
    
        }
    
      }
    }
    motor1.set(speed1);
    motor2.set(speed2);
    motor3.set(speed3);
    motor4.set(speed4);
    motor5.set(speed5);
    motor6.set(speed6);
    motor1.update();
    motor2.update();
    motor3.update();
    motor4.update();
    motor5.update();
    motor6.update();
}
 
char* subStr (char* input_string, char *separator, int segment_number) {
  char *act, *sub, *ptr;
  static char copy[MAX_STRING_LEN];
  int i;
  strcpy(copy, input_string);
  for (i = 1, act = copy; i <= segment_number; i++, act = NULL) {
    sub = strtok_r(act, separator, &ptr);
    if (sub == NULL) break;
  }
 return sub;
}
