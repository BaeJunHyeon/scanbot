/*
 * CFile1.c
 *
 * Created: 2016-07-07 오전 9:54:00
 *  Author: JooHwanSang
 */ 
#include "uart.h"

unsigned char RX0_data(void)
{
	//while(!(UCSR0A&(1<<RXC1)));
	return UDR0;
}
void TX0_data(unsigned char data0)
{
	while(!(UCSR0A&(1<<UDRE0)));
	UDR0=data0;
}
void TX0_string(unsigned char *str)
{
	int i=0;

	while(str[i] != '\0')
	{
		TX0_data(str[i]);
		i++;
	}
}

//Add by 배준현
void TX0_int(unsigned int input)
{
	char output[10];
	itoa(input, output, 10);
	int i=0;

	while(output[i] != '\0')
	{
		TX0_data(output[i]);
		i++;
	}	
}

unsigned char RX1_data(void)
{
	//while(!(UCSR0A&(1<<RXC1)));
	return UDR1;
}
void TX1_data(unsigned char data1)
{
	while(!(UCSR1A&(1<<UDRE1)));
	UDR1=data1;
}
void TX1_string(unsigned char *str)
{
	int i=0;

	while(str[i] != '\0')
	{
		TX1_data(str[i]);
		i++;
	}
}

void USART_Init(unsigned int baud)
{
	unsigned int ubrr;
	ubrr=(F_CPU/16/baud)-1;
	/* Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	/* Enable receiver and transmitter */
	UCSR0A = 0x00;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 1stop bit */
	UCSR0C = (1<<UCSZ00)|(1<<UCSZ01);
	
	/* Set baud rate */
	UBRR1H = (unsigned char)(ubrr>>8);
	UBRR1L = (unsigned char)ubrr;
	/* Enable receiver and transmitter */
	UCSR1A = 0x00;
	UCSR1B = (1<<RXEN1)|(1<<TXEN1);
	/* Set frame format: 8data, 1stop bit */
	UCSR1C = (1<<UCSZ10)|(1<<UCSZ11);
}