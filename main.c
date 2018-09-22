
#define F_CPU 16000000

#ifndef __AVR_ATmega328P__
	#define __AVR_ATmega328P__
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <util/delay.h>


#include "RTC_lib/RTC.c"

typedef struct{
	int8_t seconds;
	int8_t minutes;
	int8_t hours;
} time_t;

time_t time;

#define A_SEL 2
#define B_SEL 1
#define C_SEL 0
#define CHIP_1_SEL 4
#define CHIP_2_SEL 5
#define CHIP_3_SEL 6
#define CHIP_4_SEL 7
#define INT1_PIN 3
#define NO_SEGMENT_MASK 0x17


void EEPROM_write(unsigned int uiAddress, unsigned char ucData);
unsigned char EEPROM_read(unsigned int uiAddress);
void restore_vars_from_EEPROM();
void EEPROM_compare(unsigned int address, unsigned char data);


#define TARGET_FREQ 50
#define PRESCALER 64

//pwm pin is PB1

#define TOP_VAL  (F_CPU/(PRESCALER*TARGET_FREQ))
#define MIN_COMP (TOP_VAL/10)
#define MAX_COMP (TOP_VAL/20)
void setup_timer1(){
 // Clear Timer/Counter Control Registers
	TCCR1A = 0;
	TCCR1B = 0;

	// Set non-inverting mode on OCR1A
	TCCR1A |= (1 << COM1A1);

	// Set fast PWM Mode 14
	TCCR1A |= (1 << WGM11);
	TCCR1B |= (1 << WGM12);
	TCCR1B |= (1 << WGM13);

	//set top value 
	ICR1=TOP_VAL -1; //removing the -1 actually seems to improve

	OCR1A=MIN_COMP;

	// Set prescaler and starts PWM 
	#if PRESCALER==1
		TCCR1B |= (1 << CS10);
	#elif PRESCALER==8
		TCCR1B |= (1 << CS11);
	#elif PRESCALER==64
		TCCR1B |= (1 << CS11) | (1 << CS10);
	#endif 
}


#define SEGMENT_DELAY 300
#define SET_SEGMENT(x) OCR1A=(x)?MAX_COMP:MIN_COMP

//---------------------{  0 ,  1 ,  2 ,  3 ,  4 ,  5 ,  6 ,  7 ,  8 ,  9 ,  A ,  B ,  C ,  D ,  E ,  F , ' ',  - };
const uint8_t SEG[]= {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0x77,0x7C,0x39,0x5E,0x79,0x71,0x00,0x40};
//reversed bits because i inverted them in the layout and can't be arsed to rewrite the correct 7-SEG symbols
const uint8_t bit_reverse[7] = {0x00, 0x04, 0x02, 0x06, 0x01, 0x05, 0x03};

uint16_t display_digit(uint8_t digit_sel, uint8_t val){
	uint8_t chip_mask = 0;
	uint8_t val_mask = SEG[val];
	uint16_t millis_count=0;
	static uint8_t prev_mask[4];
	
	if (digit_sel > 4) return 0 ;
	chip_mask |= (1 << (digit_sel + 4));

	for (uint8_t i=0; i<7; i++){
		//check if the segment needs changing from its previous value
		if ((val_mask ^ prev_mask[digit_sel]) >> i){
			//set the segment value
			SET_SEGMENT( (val_mask>i) & 0x01 );
			//select the segment
			PORTD = chip_mask & bit_reverse[i] & !(1<<INT1_PIN);
			_delay_ms(SEGMENT_DELAY);
			millis_count+=SEGMENT_DELAY;

			//set to resting value to avoid glitches
			PORTD = NO_SEGMENT_MASK;
		}
	}
	
	prev_mask[digit_sel] = SEG[val];
	return millis_count; //keep track of how much time has passed from when the function was called
}


void write_time(){
	uint8_t digit0, digit1, digit2, digit3;

	digit0 = time.hours/10;
	digit1 = time.hours % 10;
	digit2 = time.minutes/10;
	digit3 = time.minutes % 10;

	display_digit(0, digit0);
	display_digit(1, digit1);
	display_digit(2, digit2);
	display_digit(3, digit3);

}

////////////////////// M A I N ////////////////////////////////////////////////////////////////

const char TIME__[] = __TIME__;
//const char DATE__[] = __DATE__;
uint8_t time_has_changed = 0;

int main(void) {
	uint8_t POWERUP_FLAGS = MCUSR; 
	MCUSR = 0;

	setup_timer1();

	//setup ports;
	DDRD = 0xf7; //0b11110111
	PORTD = NO_SEGMENT_MASK;

	//--- INT1 setup ---
	EICRA = (1<<ISC10) | (1<<ISC11);	// set INT1 on a rising edge
	EIMSK = (1<<INT1);					// enable INT1 
	DDRD &= !(1<INT1_PIN); 				// set PD3 as input
	PORTD |= (1<INT1_PIN); 				// set PD3 internal pull-up


	//enable all interrupts
	sei();

	rtc_init();

	time.seconds = atoi(&TIME__[6]) + 4;
	time.minutes = atoi(&TIME__[3]);
	time.hours = atoi(&TIME__[0]);
	
	if ((POWERUP_FLAGS>>EXTRF) & 0x01) {
		_tm.sec=time.seconds;
		_tm.min=time.minutes;
		_tm.hour=time.hours;
		rtc_set_time(&_tm);
		//enable the 1hz signal
		rtc_SQW_set_freq(FREQ_1);
		rtc_SQW_enable(true);
	}
	else rtc_get_time();

	time.seconds = (int8_t) _tm.sec;
	time.minutes = (int8_t) _tm.min;
	time.hours = (int8_t) _tm.hour;


	while (1) {
		/*
		if(time_has_changed) {
			write_time();
			time_has_changed=0;
		}
		*/

		for (int i=0; i<10; i++)
			for (int j=0; j<(2100-display_digit(0, i)); j++)
				_delay_ms(1);
		

	}
	return 0;
}


ISR(TIMER2_COMPA_vect){

}

ISR(PCINT0_vect){ 

}

// 1 second interrupt - update the time
ISR(PCINT1_vect){
	time.seconds++;
	if (time.seconds >= 60){
		time.seconds = 0;
		time.minutes++;
		time_has_changed = 1;
		if (time.minutes>=60){
			time.minutes=0;
			time.hours++;
			if (time.hours>=24){
				time.hours=0;
			}
		}
	}

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






















