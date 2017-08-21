/*
 * Servo.c
 *
 * Created: 2016-07-29 오후 4:37:18
 *  Author: JooHwanSang
 */ 

#include "Servo.h"

void servoInit()
{
	TCCR1A = (1<<COM1A1)|(0<<COM1A0)|(1<<COM1B1)|(0<<COM1B0)|(1<<COM1C1)|(0<<COM1C0)|(1<<WGM11)|(0<<WGM10);
	TCCR1B = (0<<ICNC1)|(0<<ICES1)|(1<<WGM13)|(1<<WGM12)|(0<<CS12)|(1<<CS11)|(0<<CS10);
	ICR1 = 39999;		//period 20ms / motor1,2,3
	// ICRn, OCRnx 2 = 1us
	TCCR3A = (1<<COM3A1)|(0<<COM3A0)|(1<<COM3B1)|(0<<COM3B0)|(0<<COM3C1)|(0<<COM3C0)|(1<<WGM31)|(0<<WGM30);
	TCCR3B = (0<<ICNC3)|(0<<ICES3)|(1<<WGM33)|(1<<WGM32)|(0<<CS32)|(1<<CS31)|(0<<CS30);
	ICR3 = 39999; // Tilt Motor Period 20ms
	OCR3B = 0;
	
	DDRA = 0x80;
	DDRE|=(1<<PINE3)|(1<<PINE4)|(1<<PINE5);
	DDRB = 0xE0;
}

// 배준현
void tiltInit(int Period)
{
		TCCR3A = (1<<COM3A1)|(0<<COM3A0)|(1<<COM3B1)|(0<<COM3B0)|(0<<COM3C1)|(0<<COM3C0)|(1<<WGM31)|(0<<WGM30);
		TCCR3B = (0<<ICNC3)|(0<<ICES3)|(1<<WGM33)|(1<<WGM32)|(0<<CS32)|(1<<CS31)|(0<<CS30);
		ICR3 = Period; // Tilt Motor Period 39999 = 20ms
		OCR3B = 0;
		
		DDRA = 0x80;
		DDRE|=(1<<PINE3)|(1<<PINE4)|(1<<PINE5);
		DDRB = 0xE0;


}

void servoMoveTo(unsigned char motorNum, unsigned int deg)
{
	//OCR1x 2000 ~ 4000 (0 ~ 180)
	//OCR0  9 ~ 36.69
	switch(motorNum)
	{
		case 1:
			OCR1A = deg*20+1200;
			break;
		case 2:
			OCR1B = deg*20+1200;
			break;
		case 3:
			OCR1C = deg*20+1200;
			break;
		case 4:
			OCR3A = deg*20+1200;
			break;
	}
}

void turnWheel(int m1, int m2, int m3)
{
	servoMoveTo(1,m1);
	servoMoveTo(2,m2);
	servoMoveTo(3,m3);
}

void hubMove(char dir, int speed)
{
	if(dir==1) PORTA |= (1<<PINA7);
	else if(dir==0) PORTA &= (0<<PINA7);
	
	OCR3B = speed;
}