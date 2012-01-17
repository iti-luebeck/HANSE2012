/*
 *  Robotik Praktikum WS11/12
 *  Peter Hegen, Cedric Isokeit
 *  Projekt I2C Treiber
 *  Implementierung fuer den ATMega644p
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
#include <avr/wdt.h> 

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

/*
 *Ansteuerung der einzelnen Motoren
 *Aufbau einer Verbindung zum Motorcontroller
 *und Auswahl des Motors. Senden des sollSpeed Wertes
 *mit anschliessendem schliessen der Verbindung
 */

//Ansteuerung Motor vorne (/hanse/motors/downFront)
void cbmotorfront(const hanse_msgs::sollSpeed& msg){
	nh.loginfo("test");
	Wire.beginTransmission(ADDRR);
  	Wire.send(2);
  	Wire.send(msg.data);
  	Wire.endTransmission();
}

//Ansteuerung Motor hinten (/hanse/motors/downBack)
void cbmotorback(const hanse_msgs::sollSpeed& msg){
	Wire.beginTransmission(ADDRL);
	Wire.send(2);
	Wire.send(msg.data);
	Wire.endTransmission();
}

//Ansteuerung Motor links (/hanse/motors/left)
void cbmotorleft( const hanse_msgs::sollSpeed & msg){
  	Wire.beginTransmission(ADDRL);
  	Wire.send(1);
  	Wire.send(msg.data);
  	Wire.endTransmission();
}  

//Ansteuerung Motor rechts (/hanse/motors/right)
void cbmotorright( const hanse_msgs::sollSpeed& msg){
  	Wire.beginTransmission(ADDRR);
  	Wire.send(1);
  	Wire.send(msg.data);
  	Wire.endTransmission();
}


//Definition der Subscriber fuer jeden der vier Motoren
ros::Subscriber <hanse_msgs::sollSpeed> motfront("/hanse/motors/left", &cbmotorfront);
ros::Subscriber <hanse_msgs::sollSpeed> motback("/hanse/motors/downFront", &cbmotorback);
ros::Subscriber <hanse_msgs::sollSpeed> motright("/hanse/motors/right", &cbmotorright);
ros::Subscriber <hanse_msgs::sollSpeed> motleft("/hanse/motors/downBack", &cbmotorleft);

//Definition der Message Typen
hanse_msgs::pressure press;
hanse_msgs::temperature temp;

//Definition der Publisher
ros::Publisher pubPressure("/hanse/pressure/depth", &press);
ros::Publisher pubTemperature("/hanse/pressure/temp", &temp);


//Initialisiert I2C, Nodehandler, Subscriber, Publisher und Motoren.
void setup()
{
	//Reset des Registers MCUSR und auschalten des watchdog timers	
	MCUSR = 0;
      	wdt_disable();
	//Aufbau der I2C Verbindung
  	Wire.begin();		

	//Langsames Aufleuchten der LED
	pinMode(7, OUTPUT);
	delay(200);
	digitalWrite(7,HIGH);
	delay(1000);
	digitalWrite(7,LOW);
	delay(1000);
	digitalWrite(7,HIGH);
	delay(1000);
	digitalWrite(7,LOW);

	//Initialisierung des Nodehandlers, der Subscriber und der Publisher

	nh.initNode();
	nh.subscribe(motfront);
	nh.subscribe(motback);
	nh.subscribe(motright);
	nh.subscribe(motleft);
  
   	nh.advertise(pubPressure);
   	nh.advertise(pubTemperature);
	
	//Aktivieren des watchdog timers mit parameter 2 sekunden
	wdt_enable(WDTO_2S);

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

	//Zurücksetzen des watchdog timers
	wdt_reset();


}
/*
 *loop() Lässt die LED auf dem Mikrocontroller schnell aufleuchten, wartet auf Anweisungen für die Subscriber der Motoren und published 
 *alle 200ms die Temperatur und Druck Werte des Sensors.
 */
void loop()
{
	 
	nh.spinOnce();
	digitalWrite(7,HIGH);
	delay(100);
	digitalWrite(7,LOW);
	delay(100);

	read_pressure();
  	read_temperature();

	//Zurücksetzen des watchdog timers
	wdt_reset();
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
 * Liest num Bytes von Register reg aus und schreibt diese in data. 
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
	//Aufruf der Methode i2c_read_registers. Speicherung der Sensordaten in buffer. Anzahl der gelesenen Bytes in var.
	var = i2c_read_registers(PRESSURE_TEMP_I2C_ADDR, REGISTER_PRESSURE, 2,(unsigned char*) &buffer);
	if(var!=2)
	{
		// error im Fall das weniger als 2 Byte auf dem I2C Bus gelesen wurden
		char var2[16];
		sprintf((char*)&var2,"error%d",var);
		nh.loginfo((char*)&var2);
	
	}
	else{
		//Bau einer Header msg 
		press.data = 256*buffer[0] + buffer[1]; //Darstellung der Summe von den 2 Byte Daten
		press.header.stamp = nh.now(); // Zeitstempel für den Header auf aktuelle Zeit setzen
		char temp[2] = "0";
		press.header.frame_id = temp; // Frame_id auf 0 setzen (keine Frame_id)

		//Ausgabe der Druckdaten auf der Konsole
		char var2[16];
		sprintf((char*)&var2,"%d %d",buffer[0], buffer[1]);
		nh.loginfo((char*)&var2);

		//Druckdaten werden gepublished
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
	//Aufruf der Methode i2c_read_registers. Speicherung der Sensordaten in buffer. Anzahl der gelesenen Bytes in var.
	var = i2c_read_registers(PRESSURE_TEMP_I2C_ADDR, REGISTER_TEMP, 2,(unsigned char*) &buffer);
	if(var!=2)
	{
		// error im Fall das weniger als 2 Byte auf dem I2C Bus gelesen wurden
		char var2[16];
		sprintf((char*)&var2,"error%d",var);
		nh.loginfo((char*)&var2);
	}
	else{
		//Bau einer Header msg 
		temp.data = 256*buffer[0] + buffer[1]; // Darstellung der Summe der 2 Byte Daten
		temp.header.stamp = nh.now(); // Zeitstempel auf die aktuelle Uhrzeit setzen
		char str[2] = "0"; 
		temp.header.frame_id = str; // Frame_id auf 0 setzen (keine Frame_id)

		//Ausgabe der Sensordaten des Temperatursensors auf der Konsole
		char var2[16];
		sprintf((char*)&var2,"%d %d",buffer[0], buffer[1]);
		nh.loginfo((char*)&var2);

		//Temperaturdaten werden gepublished
		pubTemperature.publish( &temp );
	}
	

}

