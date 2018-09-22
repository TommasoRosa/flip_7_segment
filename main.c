
#define F_CPU 16000000

#ifndef __AVR_ATmega328P__
	#define __AVR_ATmega328P__
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <util/delay.h>


#include "RTC_lib/RTC.c"




void EEPROM_write(unsigned int uiAddress, unsigned char ucData);
unsigned char EEPROM_read(unsigned int uiAddress);
void restore_vars_from_EEPROM();
void EEPROM_compare(unsigned int address, unsigned char data);


#define TARGET_FREQ 50
#define PRESCALER 64

//pwm pin is PB1

const TOP_VAL = F_CPU/(PRESCALER*TARGET_FREQ) ;
const MIN_COMP = TOP_VAL/10;
const MAX_COMP = TOP_VAL/20;
void setup_timer(){
 // Clear Timer/Counter Control Registers
	TCCR1A = 0;
	TCCR1B = 0;

	// Set non-inverting mode
	TCCR1A |= (1 << COM1A1);

	// Set fast PWM Mode 14
	TCCR1A |= (1 << WGM11);
	TCCR1B |= (1 << WGM12);
	TCCR1B |= (1 << WGM13);

	//set top value for 50 hz
	ICR1=TOP_VAL -1; //removing the -1 actually seems to improve

	OCR1A=MIN_COMP;

	// Set prescaler and starts PWM 
	#if PRESCALER==1
		TCCR1B = (1 << CS10);
	#elif PRESCALER==8
		TCCR1B = (1 << CS11);
	#elif PRESCALER==64
		TCCR1B = (1 << CS11) | (1 << CS10);
	#else //default keeps the timer off 
		TCCR1B = 0;
	#endif 


}

////////////////////// M A I N ////////////////////////////////////////////////////////////////

const char TIME__[] = __TIME__;
//const char DATE__[] = __DATE__;

int main(void) {

	

	//enable all interrupts
	sei();


	 //

	
	rtc_init();

	time.seconds = atoi(&TIME__[6]) + 4;
	time.minutes = atoi(&TIME__[3]);
	time.hours = atoi(&TIME__[0]);
	time.days = 6;
	
	if (0) {
		_tm.sec=time.seconds;
		_tm.min=time.minutes;
		_tm.hour=time.hours;
		_tm.wday=time.days;
		rtc_set_time(&_tm);
	}
	else rtc_get_time();

	time.seconds = (int8_t) _tm.sec;
	time.minutes = (int8_t) _tm.min;
	time.hours = (int8_t) _tm.hour;
	time.days = (int8_t) _tm.wday;


	while (1) {

	}
	return 0;
}


ISR(TIMER2_COMPA_vect){

}

ISR(PCINT0_vect){ 

}

ISR(PCINT1_vect){

}

#define _NOP() do { __asm__ __volatile__ ("nop"); } while (0)



void EEPROM_compare(unsigned int address, unsigned char data){

	uint8_t temp=EEPROM_read(address);
	if (temp!=data) EEPROM_write(address, data);
}

void EEPROM_write(unsigned int uiAddress, unsigned char ucData){
	/* Wait for completion of previous write */
	while(EECR & (1<<EEPE));
	/* Set up address and Data Registers */
	EEAR = uiAddress;
	EEDR = ucData;

	cli(); //the next two intrsuctions need to happen immediately one after the other, or else the write won't work
	/* Write logical one to EEMPE */
	EECR |= (1<<EEMPE);
	/* Start eeprom write by setting EEPE */
	EECR |= (1<<EEPE);
	sei();
}

unsigned char EEPROM_read(unsigned int uiAddress){ //should not tecnically need to be atomic
	/* Wait for completion of previous write */
	while(EECR & (1<<EEPE));
	/* Set up address register */
	EEAR = uiAddress;
	/* Start eeprom read by writing EERE */
	EECR |= (1<<EERE);
	/* Return data from Data Register */
	return EEDR;
}






















