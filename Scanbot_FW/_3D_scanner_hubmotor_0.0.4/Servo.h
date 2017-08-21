/*
 * Servo.h
 *
 * Created: 2016-07-29 오후 4:37:06
 *  Author: JooHwanSang
 */ 
#include <avr/io.h>

#ifndef SERVO_H_
#define SERVO_H_

void servoInit();
void servoMoveTo(unsigned char motorNum, unsigned int deg);
void turnWheel(int m1, int m2, int m3);
void hubMove(char dir,int speed);

#endif /* SERVO_H_ */