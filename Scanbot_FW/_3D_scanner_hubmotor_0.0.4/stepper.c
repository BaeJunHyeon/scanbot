/*
 * stepper.c
 *
 * Created: 2016-07-14 오전 11:39:12
 *  Author: JooHwanSang
 */ 
#include "stepper.h"

void stepperInit(struct STEPPER *stepper, unsigned char motorNum, unsigned char uSteps)
{
	DDRC = 0xff;		// PORTC output
	PORTC = 0x00;		// active low Enable
	
	stepper->motorNum = motorNum;
	stepper->uSteps = uSteps;	// 1/2 uSteps
	stepper->dir = 1;		//dir = 1 or -1
	stepper->steps = 0;
	stepper->flag = 0;
	
	PORTC = (0<<USM21)|(1<<USM20);
	
	/*switch(stepper->uSteps)
	{
		case 2:
		if(stepper->motorNum==1){
			PORTC |= (1<<USM10);
			PORTC &= (0<<USM11);
		}
		else if(stepper->motorNum==2){
			PORTC |= (1<<USM20);
			PORTC &= (0<<USM21);
		}
		break;
		case 4:
		if(stepper->motorNum==1){
			PORTC |= (1<<USM11);
			PORTC &= (0<<USM10);
		}
		else if(stepper->motorNum==2){
			PORTC |= (1<<USM21);
			PORTC &= (0<<USM20);
		}
		break;
		case 8:
		if(stepper->motorNum==1) PORTC |= (1<<USM11)|(1<<USM10);
		else if(stepper->motorNum==2) PORTC |= (1<<USM21)|(1<<USM20);
		break;
	}*/
}
void stepSpeed(struct STEPPER *stepper, unsigned int rpm)
{
	if(stepper->motorNum==2) OCR2 = 16000000/256/(rpm*200*2/60)/2;
}
void stepperMove(struct STEPPER *stepper, int dir)
{	
	if(stepper->motorNum==2)
	{
		timer2Token=1;
		TCCR2 = (1<<WGM21)|(0<<WGM20)|(0<<COM21)|(0<<COM20)|(1<<CS22)|(0<<CS21)|(0<<CS20);
		TIMSK = (1<<OCIE2);
		if(stepper->dir != dir)
		{
			PORTC = PORTC^(1<<DIR2);
			stepper->dir = dir;
		}
		/*if(dir==1) PORTC &= (0<<DIR2);
		else if(dir==-1) PORTC |= (1<<DIR2);*/
	}
}
