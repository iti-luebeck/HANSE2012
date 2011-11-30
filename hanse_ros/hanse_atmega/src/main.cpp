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

//Instanzieren des Nodehandlers um Puplisher und Subscriber zu erschaffen
ros::NodeHandle  nh;

/*
 *Ansteuerung der einzelnen Motoren
 *Aufbau einer Verbindung zum Motorcontroller
 *und Auswahl des Motors. Senden des sollSpeed Wertes
 *mit anschliessendem schliessen der Verbindung
 */

//Ansteuerung Motor links
void cbmotorleft( const hanse_msgs::sollSpeed & msg){
  Wire.beginTransmission(ADDRL);
  Wire.send(1);
  Wire.send(msg.data);
  Wire.endTransmission();
}  

//Ansteuerung Motor rechts
void cbmotorright( const hanse_msgs::sollSpeed& msg){
  Wire.beginTransmission(ADDRR);
  Wire.send(1);
  Wire.send(msg.data);
  Wire.endTransmission();
}

//Ansteuerung Motor hinten
void cbmotorback( const hanse_msgs::sollSpeed& msg){
  Wire.beginTransmission(ADDRL);
  Wire.send(2);
  Wire.send(msg.data);
  Wire.endTransmission();
}

//Ansteuerung Motor vorne
void cbmotorfront( const hanse_msgs::sollSpeed& msg){
  Wire.beginTransmission(ADDRR);
  Wire.send(2);
  Wire.send(msg.data);
  Wire.endTransmission();
}

//Definition der Subscriber fuer jeden der vier Motoren
ros::Subscriber <hanse_msgs::sollSpeed> motleft("/auv_name/motors/left", &cbmotorleft );
ros::Subscriber <hanse_msgs::sollSpeed> motright("/auv_name/motors/right", &cbmotorright );
ros::Subscriber <hanse_msgs::sollSpeed> motfront("/auv_name/motors/front", &cbmotorfront );
ros::Subscriber <hanse_msgs::sollSpeed> motback("/auv_name/motors/back", &cbmotorback );

hanse_msgs::pressure press;
hanse_msgs::temperature temp;

ros::Publisher pubPressure("pressure", &press);
ros::Publisher pubTemperature("temperature", &temp);

/*
 *Deklaration der Funktionen
 */
void read_pressure();
void read_temperature();

/*
 *Initialisieren des Nodehandlers, Topics auf die gelauscht werden soll,
 *Aufbau der I2c Verbindung, Init der Motorcontroller
 */
void setup()
{
  
  nh.initNode();
  nh.subscribe(motleft);
  nh.subscribe(motright);
  nh.subscribe(motback);
  nh.subscribe(motfront);

 /*
  ros::Publisher temp = nh.advertise<hanse_msgs::pressure>("/auv_name/pressure/depth", 5);
  pubPressure = &temp;
  ros::Publisher temp2 = nh.advertise<hanse_msgs::temperature>("/auv_name/pressure/temp", 5);
  pubTemperature = &temp2;
 */
  
  nh.advertise(pubPressure);
  nh.advertise(pubTemperature);

  //Blinken der LED 
  //kurzes Blinken zeigt an, dass setup() ausgefuehrt wird
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  delay(100);
  digitalWrite(10, LOW);
  delay(100);
  digitalWrite(10, HIGH);
  delay(100);
  digitalWrite(10, LOW);
  delay(100);
  
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
}

/*
 *Blinken der LED mit niedriger Frequenz zeigt an
 *dass loop() ausgefuehrt wird.
 *
 *
 */
void loop()
{
  digitalWrite(10, HIGH);

  nh.spinOnce();

  delay(500);
  digitalWrite(10, LOW);
  delay(500);

  read_pressure();
  read_temperature();
}

void i2c_scan()
{
  byte cb = 0;
  byte data = 0;
  for( int addr = 0; addr<128;addr++){
    cb = twi_writeTo(addr, &data, 0, 1);
    if(cb == 0)
     //publish i2c addr
     //7bit shift 
  	;
}
}

/*
 *Das erste Byte, dass der Master an den Slave schickt ist die Addr des Slaves.
 *Wenn sich ein Slave an der Addr befindet, wird der Slave den I2C Bus benachrichtigen.
 *Es wird 0 zurueckgegeben wenn ein Byte erfolgreich uebertragen werden konnte, ansonsten
 *wird nicht-Null zurueckgegeben. Hier werden nun die Adressen der Motorcontroller 
 *ueberprueft. Wenn die Verbindung zu den Geraeten nicht mehr besteht wird reconnect aufgerufen.
 */
void alt_i2c_scan()
{
  byte cb = 0;
  byte data = 0;
  cb = twi_writeTo(ADDRL, &data, 0, 1);
  if(cb == 0){
  	cb = twi_writeTo(ADDRR, &data, 0, 1);
     	if(cb == 0){

	}else{
		//reconnect i2c || state error
	}
  }else{
	//reconnect i2c || state error
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
	unsigned char buffer[2];
	if(2 != i2c_read_registers(PRESSURE_TEMP_I2C_ADDR, REGISTER_PRESSURE, 2,(unsigned char*) &buffer))
	{
		// error
	}
	else
	{
		hanse_msgs::pressure msg;
		msg.data = 256*buffer[0] + buffer[1];
		pubPressure.publish( &msg );
	}
}

/*
 * Liesst die Temperatur aus und schickt sie über ROS.
 */
void read_temperature()
{
	unsigned char buffer[2];
	if(2 != i2c_read_registers(PRESSURE_TEMP_I2C_ADDR, REGISTER_TEMP, 2,(unsigned char*) &buffer))
	{
		// error
	}
	else
	{
		hanse_msgs::temperature msg;
		msg.data = 256*buffer[0] + buffer[1];
		pubTemperature.publish( &msg );
	}
}
