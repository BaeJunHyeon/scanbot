/*
 * uart.h
 *
 * Created: 2016-07-07 오전 9:50:53
 *  Author: JooHwanSang
 */ 
#include <avr/io.h>

#ifndef UART_H_
#define UART_H_

#define F_CPU 16000000UL

unsigned char RX0_data(void);
void TX0_data(unsigned char data);
void TX0_string(unsigned char *str);
unsigned char RX1_data(void);
void TX1_data(unsigned char data);
void USART_Init(unsigned int baud);
void TX0_string(unsigned char *str);
void TX0_int(unsigned int input);
#endif /* UART_H_ */