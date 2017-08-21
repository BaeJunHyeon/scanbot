/*
 * ultrasonic.h
 *
 * Created: 2016-08-01 오전 9:59:21
 *  Author: JooHwanSang
 */ 
#include <avr/io.h>
#include <util/delay.h>

#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

void sensorInit(void);
void checkDist(void);


#endif /* ULTRASONIC_H_ */