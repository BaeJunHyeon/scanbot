/*
 * stepper.h
 *
 * Created: 2016-07-14 오전 11:38:01
 *  Author: JooHwanSang
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include "uart.h"

#ifndef STEPPER_H_
#define STEPPER_H_

#define ENABLE 7
#define DIR1 6
#define DIR2 2
#define USM10 5
#define USM11 4
#define USM20 1
#define USM21 0

volatile unsigned char timer1Token;
volatile unsigned char timer2Token;

struct STEPPER
{
	unsigned char motorNum;
	unsigned char uSteps;
	int dir;
	long stepCnt;
	long steps;
	volatile unsigned long uSec_step;
	volatile unsigned long sCnt;
	volatile unsigned char flag;
	volatile unsigned long freq_step;
};

void stepperInit(struct STEPPER *stepper, unsigned char motorNum, unsigned char uSteps);
void stepSpeed(struct STEPPER *stepper, unsigned int rpm);
void stepperMove(struct STEPPER *stepper, int dir);


#endif /* STEPPER_H_ */