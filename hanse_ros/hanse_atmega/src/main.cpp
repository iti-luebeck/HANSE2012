/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <WProgram.h>
#include <ros.h>
#include <Wire.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include "hanse_msgs/sollSpeed.h"
#include "utility/twi.h"

//M2 Motor hinten, M1 Motor links
#define ADDRL 0xBE>>1
//M2 Motor vorne , M1 Motor rechts
#define ADDRR 0xB0>>1

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

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


void cbmotorback( const hanse_msgs::sollSpeed& msg){
  Wire.beginTransmission(ADDRL);
  Wire.send(2);
  Wire.send(msg.data);
  Wire.endTransmission();
}


void cbmotorfront( const hanse_msgs::sollSpeed& msg){
  Wire.beginTransmission(ADDRR);
  Wire.send(2);
  Wire.send(msg.data);
  Wire.endTransmission();
}

ros::Subscriber <hanse_msgs::sollSpeed> motleft("/auv_name/motors/left", &cbmotorleft );
ros::Subscriber <hanse_msgs::sollSpeed> motright("/auv_name/motors/right", &cbmotorright );
ros::Subscriber <hanse_msgs::sollSpeed> motfront("/auv_name/motors/front", &cbmotorfront );
ros::Subscriber <hanse_msgs::sollSpeed> motback("/auv_name/motors/back", &cbmotorback );

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

void i2c_scan()
{
  byte cb = 0;
  byte data = 0;
  for( int addr = 0; addr<128;addr++){
    cb = twi_writeTo(addr, &data, 0, 1);
    if(cb == 0)
     //publish i2c addr
     //7bit shift ->
	;
  }
}

#define PRESSURE_TEMP_I2C_ADDR 0x50<<1

#define REGISTER_CALIB 0
#define REGISTER_PRESSURE_RAW 8
#define REGISTER_TEMP_RAW 10
#define REGISTER_PRESSURE 12
#define REGISTER_TEMP 14
#define REGISTER_STATUS 17
#define REGISTER_COUNTER 20

#define STATUS_MAGIC_VALUE 0x55
#define CALIB_MAGIC_VALUE 224

int i2c_read_registers(unsigned char addr, unsigned char reg, int num, unsigned char* data)
{
	int _num = 0;

	Wire.beginTransmission(addr);
	Wire.send(reg);
	Wire.endTransmission();
	Wire.beginTransmission(addr);
	Wire.requestFrom(addr, num);
	for(_num=0; _num<num && Wire.available(); _num++)
	{
		data[_num] = Wire.receive();
	}
	Wire.endTransmission();

	return _num;
}

void read_pressure()
{
	unsigned char buffer[2];
	if(2 != i2c_read_registers(PRESSURE_TEMP_I2C_ADDR, REGISTER_PRESSURE, 2, &buffer))
	{
		// error
	}
	else
	{
		int res = 256*buffer[0] + buffer[1];
		// success, TODO: publish
	}
}

void read_temperature()
{
	unsigned char buffer[2];
	if(2 != i2c_read_registers(PRESSURE_TEMP_I2C_ADDR, REGISTER_TEMP, 2, &buffer))
	{
		// error
	}
	else
	{
		int res = 256*buffer[0] + buffer[1];
		// success, TODO: publish
	}
}
