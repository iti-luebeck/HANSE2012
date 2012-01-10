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

void read_pressure();
void read_temperature();

void cbmotorfront(const hanse_msgs::sollSpeed& msg){
	nh.loginfo("test");
	Wire.beginTransmission(ADDRR);
  	Wire.send(2);
  	Wire.send(msg.data);
  	Wire.endTransmission();
}

void cbmotorback(const hanse_msgs::sollSpeed& msg){
	Wire.beginTransmission(ADDRL);
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
	//Aufbau der I2C Verbindung
  	Wire.begin();
	pinMode(7, OUTPUT);
	delay(200);
	digitalWrite(7,HIGH);
	delay(1000);
	digitalWrite(7,LOW);
	delay(1000);
	digitalWrite(7,HIGH);
	delay(1000);
	digitalWrite(7,LOW);

	nh.initNode();
	nh.subscribe(motfront);
	nh.subscribe(motback);
	nh.subscribe(motright);
	nh.subscribe(motleft);
  
   	nh.advertise(pubPressure);
   	nh.advertise(pubTemperature);
	
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

	nh.loginfo("setup");

}

void loop()
{

	nh.spinOnce();
	digitalWrite(7,HIGH);
	delay(100);
	digitalWrite(7,LOW);
	delay(100);
	read_pressure();
  	read_temperature();
}


#define PRESSURE_TEMP_I2C_ADDR 0x50>>1

#define REGISTER_CALIB 0
#define REGISTER_PRESSURE_RAW 8
#define REGISTER_TEMP_RAW 10
#define REGISTER_PRESSURE 12
#define REGISTER_TEMP 14
#define REGISTER_STATUS 17
#define REGISTER_COUNTER 20

#define STATUS_MAGIC_VALUE 0x55
#define CALIB_MAGIC_VALUE 224

/*
 * Liesst num Bytes von Register reg aus und schreibt diese in data. 
 * data muss bereits vorinitialisiert sein. Gibt die Anzahl der tatsächlich gelesenen Bytes zurück
 */

int i2c_read_registers(unsigned char addr, unsigned char reg, int num, unsigned char* data)
{
	int _num = 0;

	Wire.beginTransmission(addr);
	Wire.send(reg);
	Wire.endTransmission();
	Wire.beginTransmission(addr);
	Wire.requestFrom((int)addr, num);
	for(_num=0; _num<num && Wire.available(); _num++)
	{
		data[_num] = Wire.receive();
	}
	Wire.endTransmission();

	return _num;
}

/*
 * Liesst den Druck aus und schickt ihn über ROS.
 */

void read_pressure()
{
	unsigned int var;
	unsigned char buffer[2];
	var = i2c_read_registers(PRESSURE_TEMP_I2C_ADDR, REGISTER_PRESSURE, 2,(unsigned char*) &buffer);
	if(var!=2)
	{
		// error
		char var2[16];
		sprintf((char*)&var2,"error%d",var);
		nh.loginfo((char*)&var2);
	
	}
	else{
		press.data = 256*buffer[0] + buffer[1];
		press.header.stamp = nh.now();
		char temp[2] = "0";
		press.header.frame_id = temp;

		char var2[16];
		sprintf((char*)&var2,"%d %d",buffer[0], buffer[1]);
		nh.loginfo((char*)&var2);
		pubPressure.publish( &press );
	}

	
}

/*
 * Liesst die Temperatur aus und schickt sie über ROS.
 */


void read_temperature()
{
	unsigned int var;
	unsigned char buffer[2];
	var = i2c_read_registers(PRESSURE_TEMP_I2C_ADDR, REGISTER_TEMP, 2,(unsigned char*) &buffer);
	if(var!=2)
	{
		// error
		char var2[16];
		sprintf((char*)&var2,"error%d",var);
		nh.loginfo((char*)&var2);
	}
	else{
		temp.data = 256*buffer[0] + buffer[1];
		temp.header.stamp = nh.now();
		char str[2] = "0";
		temp.header.frame_id = str;

		char var2[16];
		sprintf((char*)&var2,"%d %d",buffer[0], buffer[1]);
		nh.loginfo((char*)&var2);
		pubTemperature.publish( &temp );
	}
	

}

