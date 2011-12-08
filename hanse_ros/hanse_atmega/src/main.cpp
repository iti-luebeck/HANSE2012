/*
 *  Robotik Praktikum WS11/12
 *  Peter Hegen, Cedric Isokeit
 *  Projekt I2C Treiber
 *  Implementierung fuer den ATMega168
 *  Ansteuerung der Motorcontroller und des Drucksensors
 */


#include <WProgram.h>
#include <ros.h>
#include <Wire.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include "hanse_msgs/sollSpeed.h"
#include "hanse_msgs/pressure.h"
#include "hanse_msgs/temperature.h"
#include "utility/twi.h"

/*
 *Adresse eines Motorcontrollers
 *M1: Motor links, M2: Motor hinten
 */
#define ADDRL 0xBE>>1
/*
 *Adresse eines Motorcontrollers
 *M1: Motor rechts, M2: Motor vorne
 */
#define ADDRR 0xB0>>1


ros::NodeHandle nh;

void cbmotorfront(const hanse_msgs::sollSpeed& msg){
	Wire.beginTransmission(ADDRR);
  	Wire.send(2);
  	Wire.send(msg.data);
  	Wire.endTransmission();
}

void cbmotorback(const hanse_msgs::sollSpeed& msg){
	Wire.beginTransmission(ADDRR);
	Wire.send(2);
	Wire.send(msg.data);
	Wire.endTransmission();
}

void cbmotorleft( const hanse_msgs::sollSpeed & msg){
  Wire.beginTransmission(ADDRL);
  Wire.send(1);
  Wire.send(msg.data);
  Wire.endTransmission();
}  

void cbmotorright( const hanse_msgs::sollSpeed& msg){
  Wire.beginTransmission(ADDRR);
  Wire.send(1);
  Wire.send(msg.data);
  Wire.endTransmission();
}


ros::Subscriber <hanse_msgs::sollSpeed> motfront("front", &cbmotorfront);
ros::Subscriber <hanse_msgs::sollSpeed> motback("back", &cbmotorback);
ros::Subscriber <hanse_msgs::sollSpeed> motright("right", &cbmotorright);
ros::Subscriber <hanse_msgs::sollSpeed> motleft("left", &cbmotorleft);

hanse_msgs::pressure press;
hanse_msgs::temperature temp;

ros::Publisher pubPressure("pressure", &press);
ros::Publisher pubTemperature("temperature", &temp);

void setup()
{

	nh.initNode();
	nh.subscribe(motfront);
	nh.subscribe(motback);
	nh.subscribe(motright);
	nh.subscribe(motleft);

  
   	nh.advertise(pubPressure);
   	nh.advertise(pubTemperature);
	
	//Aufbau der I2C Verbindung
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

	pinMode(7, OUTPUT);
	delay(200);
	digitalWrite(7,HIGH);
	delay(1000);
	digitalWrite(7,LOW);
	delay(1000);
	digitalWrite(7,HIGH);
	delay(1000);
	digitalWrite(7,LOW);
}

void loop()
{
	nh.spinOnce();
	digitalWrite(7,HIGH);
	delay(200);
	digitalWrite(7,LOW);
	delay(200);
}
