/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <WProgram.h>
#include <ros.h>
#include <Wire.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>


//M2 Motor hinten, M1 Motor links
#define ADDRL 0xBE>>1
//M2 Motor vorne , M1 Motor rechts
#define ADDRR 0xB0>>1

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

void cbmotorleft( const std_msgs::Int8& msg){
  Wire.beginTransmission(ADDRL);
  Wire.send(1);
  Wire.send(msg.data);
  Wire.endTransmission();
}  

void cbmotorright( const std_msgs::Int8& msg){
  Wire.beginTransmission(ADDRR);
  Wire.send(1);
  Wire.send(msg.data);
  Wire.endTransmission();
}


void cbmotorback( const std_msgs::Int8& msg){
  Wire.beginTransmission(ADDRL);
  Wire.send(2);
  Wire.send(msg.data);
  Wire.endTransmission();
}


void cbmotorfront( const std_msgs::Int8& msg){
  Wire.beginTransmission(ADDRR);
  Wire.send(2);
  Wire.send(msg.data);
  Wire.endTransmission();
}

ros::Subscriber <std_msgs::Int8> motleft("/auv_name/motors/left", &cbmotorleft );
ros::Subscriber <std_msgs::Int8> motright("/auv_name/motors/right", &cbmotorright );
ros::Subscriber <std_msgs::Int8> motfront("/auv_name/motors/front", &cbmotorfront );
ros::Subscriber <std_msgs::Int8> motback("/auv_name/motors/back", &cbmotorback );

char hello[13] = "hello world!";



void setup()
{
  
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(motleft);
  nh.subscribe(motright);
  nh.subscribe(motback);
  nh.subscribe(motfront);
  
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  delay(100);
  digitalWrite(10, LOW);
  delay(100);
  digitalWrite(10, HIGH);
  delay(100);
  digitalWrite(10, LOW);
  delay(100);
  
  Wire.begin();
  //INIT Motorcontroller links Umstellung auf signed int
  Wire.beginTransmission(ADDRL);
  Wire.send(0);
  Wire.send(1);
  Wire.endTransmission();
  //INIT Motorcontroller rechts Umstellung auf signed int
  Wire.beginTransmission(ADDRR);
  Wire.send(0);
  Wire.send(1);
  Wire.endTransmission();
}


void loop()
{
  digitalWrite(10, HIGH);
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  //Serial.print('A');
  delay(500);
  digitalWrite(10, LOW);
  delay(500);

  
}
