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
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/KeyValue.h>
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

/*
 *Definition der I2C Adresse des Drucksensors, sowie
 *der Register Adressen.
 */
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



// Definition des NodeHandle und der Funktionen
ros::NodeHandle nh;

void read_pressure();
void read_temperature();
void diagnostics();
int i2c_read_registers(unsigned char addr, unsigned char reg, int num, unsigned char* data);
void i2c_write(unsigned char addr, unsigned char reg, const hanse_msgs::sollSpeed& msg);

/*
 *disconnect_timer ist ein counter für den test ob die serial_node noch verbunden ist
 *connected ist eine variable ob die verbindung mit der serial_node aufgebaut wurde
 *error_x sind variablen die sich merken ob ein Fehler aufgetreten ist
 */

//timer Variable für den Test ob die serial_node noch verbunden ist
char disconnect_timer = 0;
//timer Variable für die Ausgabe der Fehlermeldung nach wdt reset. Fehlermeldung wird eine gewisse Zeit gehalten,
//da sie ansonsten nach Neustart der serial_Node schon wieder zurückgesetzt wurde
char diagnostic_timer = 0;
//connected = 0 : bisher keine Verbindung zur Serial_node, connected = 1 : Verbindung zur Serial_node wurde hergestellt
char connected = 0;
//error_motor = 0 : kein Fehler beim Schreiben auf die Motoren festgestellt, error_motor = 1 : Fehler beim Schreiben festgestellt
char error_motor = 0;
//error_depth = 0 : kein Fehler, 2 Bytes vom Druck Sensor erhalten, error_depth = 1 : Fehler, weniger als 2 Bytes erhalten
char error_depth = 0;
//error_temp = 0 : kein Fehler, 2 Bytes vom Druck Sensor erhalten, error_temp = 1 : Fehler, weniger als 2 Bytes erhalten
char error_temp = 0;
//press_val speichert den letzten ausgelesenen Druck Wert für die Diagnostic Nachrichten
int  press_val;
//temp_val speichert den letzten ausgelesenen Temperatur Wert für die Diagnostic Nachrichten
int  temp_val;
//mcusr_mirror speichert den Zustand des MCU Status Registers um später die Reset Ursache des Controllers auswerten zu können
unsigned char mcusr_mirror;

/*
 *Ansteuerung der einzelnen Motoren
 *Aufbau einer Verbindung zum Motorcontroller
 *und Auswahl des Motors. Senden des sollSpeed Wertes
 *mit anschliessendem schliessen der Verbindung
 */

//Ansteuerung Motor vorne (/hanse/motors/downFront)
void cbmotorfront(const hanse_msgs::sollSpeed& msg){

        i2c_write(ADDRR, 2, msg);
}

//Ansteuerung Motor hinten (/hanse/motors/downBack)
void cbmotorback(const hanse_msgs::sollSpeed& msg){

        i2c_write(ADDRL, 2, msg);
}

//Ansteuerung Motor links (/hanse/motors/left)
void cbmotorleft( const hanse_msgs::sollSpeed & msg){

        i2c_write(ADDRL, 1, msg);
}  

//Ansteuerung Motor rechts (/hanse/motors/right)
void cbmotorright( const hanse_msgs::sollSpeed& msg){

        i2c_write(ADDRR, 1, msg);
}


/*
 *Liest den Wert vom Motor aus und lieftert ihn zurück
 *      addr : Adresse des Motors
 *      reg : Register aus dem gelesen wird
 *returns Daten des Motors
 */
signed char motor_value(unsigned char addr, unsigned char reg)
{
    char data;
    i2c_read_registers(addr, reg, 1,(unsigned char*) &data);
    return data;
}


//Definition der Subscriber fuer jeden der vier Motoren
ros::Subscriber <hanse_msgs::sollSpeed> motleft("/hanse/motors/left", &cbmotorleft);
ros::Subscriber <hanse_msgs::sollSpeed> motfront("/hanse/motors/downFront", &cbmotorfront);
ros::Subscriber <hanse_msgs::sollSpeed> motright("/hanse/motors/right", &cbmotorright);
ros::Subscriber <hanse_msgs::sollSpeed> motback("/hanse/motors/downBack", &cbmotorback);



//Definition der Message Typen
hanse_msgs::pressure press;
hanse_msgs::temperature temp;
diagnostic_msgs::DiagnosticArray diag_array;



//Definition der Publisher
ros::Publisher pubPressure("/hanse/pressure/depth", &press);
ros::Publisher pubTemperature("/hanse/pressure/temp", &temp);
ros::Publisher pubStatus("/diagnostics", &diag_array);


//Initialisiert I2C, Nodehandler, Subscriber, Publisher und Motoren.
void setup()
{
        //Sichern des Registers MCUSR zum Auswerten der Reset Ursache
        mcusr_mirror = MCUSR;
        //Reset des Registers MCUSR und auschalten des watchdog timers
        MCUSR = 0;
      	wdt_disable();

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

        //Aktivierung des watchdogtimer um sich aus Deadlocks von i2c zu retten
        wdt_enable(WDTO_2S);

        //Initialisierungen für die i2c Verbindung
        Wire.begin();


	//Initialisierung des Nodehandlers, der Subscriber und der Publisher

	nh.initNode();
	nh.subscribe(motfront);
	nh.subscribe(motback);
	nh.subscribe(motright);
	nh.subscribe(motleft);
  
   	nh.advertise(pubPressure);
   	nh.advertise(pubTemperature);
	nh.advertise(pubStatus);	




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

        //Definition des msg Typs
        hanse_msgs::sollSpeed msg;
        //Setzen der Sollgeschwindigkeit auf 0
        msg.data= 0;

        //Ansteuerung der Thruster: Zurücksetzen der Geschwindigkeit auf 0
        //damit Werte die eventuell noch in dem Motor Controllern gespeichert sind auf 0 gesetzt werden
        cbmotorfront(msg);
        cbmotorback(msg);
        cbmotorleft(msg);
        cbmotorright(msg);



	nh.loginfo("setup");


	wdt_reset();


}
/*
 *loop() Lässt die LED auf dem Mikrocontroller schnell aufleuchten, wartet auf Anweisungen für die Subscriber der Motoren und published 
 *alle 200ms die Temperatur und Druck Werte des Sensors. Wenn Probleme mit dem Delay auftauchen können die Delay Werte angepasst werden
 */
void loop()
{
	 
	nh.spinOnce();
        digitalWrite(7,HIGH);
	delay(100);
	digitalWrite(7,LOW);
	delay(100);

        //Auslesen des Drucks un der Temperatur und publishen
	read_pressure();
  	read_temperature();

	wdt_reset();

        //Bau der Diagnostic Nachricht und publishen
        diagnostics();

        /*
         *Der Wert von mcusr_mirror wird nach 10 Durchläufen von loop() auf 2 gesetzt (normaler Reset)
         *damit bei einem wdt reset die Fehlermeldung auch auf dem Robot Monitor erscheint und nicht
         *verloren geht, weil die serial_node zu lange gebraucht hat um neu zu connecten
         */
        if(diagnostic_timer==10){

            diagnostic_timer = 0;
            mcusr_mirror = 2;
        }

        /*
         *Sobald eine Verbindung mit der Serial_node besteht, wird connected auf 1
         * gesetzt und der disconnect_timer auf 0 gesetzt.  Wenn nun die Verbindung abbricht
         * wird der Wert für den disconnect_timer nicht mehr zurückgesetzt und die andere
         * if Abfrage für das Auftauchen wird erfüllt
         */
        if(nh.connected()){
                //nh.loginfo("connected");
                connected = 1;
                disconnect_timer = 0;
        }

        /*
         *Sobald keine Verbindung zur Serial_node mehr besteht und eine gewisse Zeit
         *abgelaufen ist, werden die Motoren so angeseteuert, dass das auv auftaucht.
         */
        if(!nh.connected()&&(disconnect_timer>10)&&connected){

                //Definition des msg Typs
                hanse_msgs::sollSpeed msg;
                //Setzen der Sollgeschwindigkeit auf 110
                msg.data= 110;

                //Ansteuerung der Thruster vorne und hinten
                cbmotorfront(msg);
                cbmotorback(msg);

                //reset der Variable connected
                connected = 0;
        }

        disconnect_timer++;

        //Diagnostic timer hochzählen für den reset des mcusr_mirror
        if(nh.connected()){
                diagnostic_timer++;
        }

}




/*
 *i2c_write schreibt per i2c die Daten der hanse_msgs::sollSpeed auf das Register der angegebenen Adresse
 *param:
 *  addr: Zieladresse des Motorcontrollers
 *  reg: Zielregister des Motorcontrollers
 *  msg: hanse_msgs::sollSpeed enthält die Sollgeschwindigkeit
 */

void i2c_write(unsigned char addr, unsigned char reg, const hanse_msgs::sollSpeed& msg)
{
        Wire.beginTransmission(addr);
        Wire.send(reg);
        Wire.send(msg.data);
        //Wenn etwas anderes als 0 := ACK zurückgegeben wird, ist ein Fehler aufgetreten.
        if(Wire.endTransmission()>0){
            error_motor = 1;
        }
 }

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
 * Auslesen des ATmega, der den Druck-/Temperatursensor ansteuert.
 */
int16_t read_temp_pressure(uint8_t addr, uint8_t reg)
{
	unsigned int var;
	unsigned char buffer[2];

	//Aufruf der Methode i2c_read_registers. Speicherung der Sensordaten in buffer. Anzahl der gelesenen Bytes in var.
	var = i2c_read_registers(addr, reg, 2,(unsigned char*) &buffer);
	if(var!=2)
	{
		// error im Fall das weniger als 2 Byte auf dem I2C Bus gelesen wurden (Konsolenausgabe)
		char var2[16];
                sprintf((char*)&var2,"error press%d",var);
		nh.loginfo((char*)&var2);

		// Fehler. Druck und Temperatur dürften im Wasser nicht negativ werden.
		return -1;
	}
	
	return 256*buffer[0] + buffer[1];
}

/*
 * Liesst den Druck aus und schickt ihn über ROS.
 */
void read_pressure()
{
	int16_t ret = read_temp_pressure(PRESSURE_TEMP_I2C_ADDR, REGISTER_PRESSURE);
	if(ret < 0)
	{
		error_depth = 1;
		return;
	}

	//Bau einer Header msg 
	press.data = ret;
	press.header.stamp = nh.now(); // Zeitstempel für den Header auf aktuelle Zeit setzen
	char temp[2] = "0";
	press.header.frame_id = temp; // Frame_id auf 0 setzen (keine Frame_id)

	//Druckdaten werden gepublished
	press_val = press.data;
	pubPressure.publish( &press );
}

/*
 * Liesst die Temperatur aus und schickt sie über ROS.
 */
void read_temperature()
{
	int16_t ret = read_temp_pressure(PRESSURE_TEMP_I2C_ADDR, REGISTER_TEMP);
	if(ret < 0)
	{
		error_temp = 1;
		return;
	}

	//Bau einer Header msg 
	temp.data = ret;
	temp.header.stamp = nh.now(); // Zeitstempel auf die aktuelle Uhrzeit setzen
	char str[2] = "0"; 
	temp.header.frame_id = str; // Frame_id auf 0 setzen (keine Frame_id)

	//Temperaturdaten werden gepublished
	temp_val = temp.data;
	pubTemperature.publish( &temp );
}




/*
 *diagnostics published ein diagnostic_array für den Status der Motoren und
 *des Druck Sensors. Ausserdem wird die letzte Reset Ursache des Controllers
 *mitgesendet
 */
void diagnostics()
{

        //Definition der msg Typen
        diagnostic_msgs::DiagnosticStatus status_msg[3];
        diagnostic_msgs::KeyValue kv1[1];
        diagnostic_msgs::KeyValue kv2[4];
        diagnostic_msgs::KeyValue kv3[2];

        //Definition von Variablen für die Pointer
        char var1[16];
        char var2[16];
        char var3[16];
        char var4[16];
        char var5[16];
        char var6[16];
        char var7[16];

        //Definition des Headers des diagnostic_arrays
        diag_array.header.stamp = nh.now();
        diag_array.header.frame_id = "";

        //Setzen der Länge auf 3 für 3 Nachrichten
        diag_array.status_length = 3;




        /*
         *Reset Ursache des Controllers
         *Wenn der Wert des Registers mcusr auf 8 stand wird der Level auf 2 also error gesetzt
         *und in den Keyvalues wird nochmal der Wert des mcusr übergeben.
         */
        if(mcusr_mirror == 8){
            status_msg[0].level = 2;
        }else{
            status_msg[0].level = 0;
        }


        //Bau der Status msg
        status_msg[0].name ="diag_reset";
        status_msg[0].message ="";
        status_msg[0].hardware_id = "0";
        status_msg[0].values_length = 1;

        //Zuweisung der Key values
        kv1[0].key ="MCUSR";
        sprintf((char*)&var1,"%d",mcusr_mirror);
        kv1[0].value=var1;

        status_msg[0].values = kv1;

        /*
         *Diagnose der Motoren
         *Wenn beim Schreiben auf die Motoren von twi_endtransmission ein Fehler festgestellt wurde
         *wird level auf 2 also auf error gesetzt, außerdem wird in den Keyvalues die entsprechenden
         *Werte der Motoren geschrieben
         */
        if(error_motor==1){
            status_msg[1].level = 2;
            error_motor = 0;
        }else{
            status_msg[1].level = 0;
        }

        //Bau der Status msg
        status_msg[1].name ="diag_motoren";
        status_msg[1].message ="";
        status_msg[1].hardware_id = "1";
        status_msg[1].values_length = 4;

        //Zuweisung der Key values
        // /hanse/motors/left
        kv2[0].key ="left";
        sprintf((char*)&var2,"%d",motor_value(ADDRL,1));
        kv2[0].value=var2;

        // /hanse/motors/right
        kv2[1].key ="right";
        sprintf((char*)&var3,"%d",motor_value(ADDRR,1));
        kv2[1].value=var3;

        // /hanse/motors/downFront
        kv2[2].key ="front";
        sprintf((char*)&var4,"%d",motor_value(ADDRR,2));
        kv2[2].value=var4;

        // /hanse/motors/downBack
        kv2[3].key ="back";
        sprintf((char*)&var5,"%d",motor_value(ADDRL,2));
        kv2[3].value=var5;

        status_msg[1].values = kv2;


        /*
        *Diagnose Druck Sensor
        *Wenn weniger als 2 Byte übertragen wurden, wird level auf 2 also error gesetzt
        *und in den Key Values werden die Werte der Sensoren übergeben
        */
        if(error_depth == 1 || error_temp == 1){
            status_msg[2].level = 2;
            error_depth = 0;
            error_temp = 0;
        }else{
            status_msg[2].level = 0;
        }

        //Bau der Status msg
        status_msg[2].name ="diag_druck";
        status_msg[2].message ="";
        status_msg[2].hardware_id = "2";
        status_msg[2].values_length = 2;

        //Zuweisung der Key Values

        // Druck Daten
        kv3[0].key ="Druck";
        sprintf((char*)&var6,"%d",press_val);
        kv3[0].value=var6;

        // Temperatur Daten
        kv3[1].key ="Temperatur";
        sprintf((char*)&var7,"%d",temp_val);
        kv3[1].value=var7;

        status_msg[2].values = kv3;



        diag_array.status = status_msg;

        pubStatus.publish( &diag_array);




}
