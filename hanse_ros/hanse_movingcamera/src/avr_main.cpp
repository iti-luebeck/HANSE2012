#define F_CPU 16000000UL
#include "ros.h"
#include "std_msgs/Int16.h"
#include <avr/wdt.h> 
#include <avr/io.h>
#include <util/delay.h>
// Include C headers (ie, non C++ headers) in this block
extern "C"
{
  #include <util/delay.h>
}

// Needed for AVR to use virtual functions
extern "C" void __cxa_pure_virtual(void);
void __cxa_pure_virtual(void) {}

// globale Variablen
volatile int16_t pwmValue = 375;           

ros::NodeHandle nh;

// Timer 1 Output COMPARE A Interrupt
ISR(TIMER1_COMPA_vect) {
	OCR1A = pwmValue;
}

/**
 * Write new data into OCR1A
 */
void Callback(const std_msgs::Int16& int_msg)
{ 	
  	//Calculate values for prescaler 64
	if (int_msg.data <= -90) {
		pwmValue = 150;
	} else if (int_msg.data >= 90) {
		pwmValue = 600;
	} else {
		pwmValue = (int) (2.5 * int_msg.data + 375);
	}
	
	OCR1A = pwmValue;
}

// Initialize ROS Subscriber
ros::Subscriber<std_msgs::Int16> sub("hanse_camera_pos", &Callback );

int main()
{
   //Configure TIMER1
   TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);        //NON Inverted PWM
   TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10); //PRESCALER=64 MODE 14(FAST PWM)

   ICR1=4999; 				// fPWM=50Hz (Period = 20ms Standard).
   OCR1A = pwmValue;		// default value for pwm
   DDRD|=(1<<PD5);			// PWM Pins as Out
   TIMSK1 |= (1<<OCIE1A);	// Interrupt freischalten
   
  // LED initialisieren
  DDRD = DDRD | (1<<PD7);
  // Und ausschalten
  PORTD |= (1<<PD7);
  
  nh.initNode();
  nh.subscribe(sub);
  
  while(1)
  {
  	nh.spinOnce();
  }
  
  return 0;
}

