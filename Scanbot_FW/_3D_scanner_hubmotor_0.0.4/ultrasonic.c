/*
 * ultrasonic.c
 *
 * Created: 2016-08-01 오전 9:59:42
 *  Author: JooHwanSang
 */ 
#define F_CPU 16000000UL
#include "ultrasonic.h"

void sensorInit()
{
	DDRE |= (1<<PINE2);
	
	EICRB = (1<<ISC71)|(0<<ISC70)|(1<<ISC61)|(1<<ISC60);	//INT6 rising edge, INT7 falling edge
	EIMSK = (1<<INT7)|(1<<INT6);
	
	TCCR0 = (1<<WGM01)|(0<<WGM00)|(0<<COM01)|(0<<COM00)|(0<<CS02)|(1<<CS01)|(0<<CS00);
	TIMSK = (0<<OCIE0);
	OCR0 = 19;
}
void checkDist()
{
	PORTE |= (1<<PINE2);
	_delay_us(10);
	PORTE &= (0<<PINE2);
	_delay_ms(50);
}