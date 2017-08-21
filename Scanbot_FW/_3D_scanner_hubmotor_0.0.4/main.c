
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "uart.h"
#include "Servo.h"
#include "stepper.h"
#include "ultrasonic.h"

volatile unsigned int sonicCNT;
volatile unsigned int dist;
volatile unsigned char sonicFlag;
volatile unsigned char timer1Token;
volatile unsigned char timer2Token;
unsigned char isReady=0;

unsigned char data;		//RX buffer

unsigned int servo1deg=3;		//rear servo
unsigned int servo2deg=164;		//front1 servo 90'=72
unsigned int servo3deg=18;		//front2 servo 90'=158
unsigned int servo4deg=90-20;		//tilt servo

struct STEPPER headMotor;

ISR(TIMER2_COMP_vect)		//motor2 & UltraSonic sensor count
{
	PORTE = PORTE^(timer2Token<<PINE5);
	headMotor.stepCnt += headMotor.dir*timer2Token;
	headMotor.steps = headMotor.stepCnt/2;
}
ISR(TIMER0_COMP_vect)
{
	sonicCNT++;
}
ISR(INT6_vect)		//UltraSonic rising edge
{
	TIMSK |= (1<<OCIE0);
	OCR0 = 19;
}
ISR(INT7_vect)		//UltraSonic falling edge
{
	TIMSK &= (0<<OCIE0);
	dist = sonicCNT/6;
	sonicCNT = 0;
}

int main(void)
{
	cli();
	
	stepperInit(&headMotor,2,2);
	sensorInit();
	
	servoInit();
	servoMoveTo(1,servo1deg);
	servoMoveTo(2,servo2deg);
	servoMoveTo(3,servo3deg);
	servoMoveTo(4,servo4deg);
	
	USART_Init(9600);
	sonicCNT=0;
	sonicFlag=0;
	dist=0;
	unsigned int setDist=80;
	unsigned int headRPM=80;
	int hubSpeed=0;
	
	unsigned int headHeight=1600;
	unsigned int heightEx=0;
	unsigned int heightMM=100;
	
	unsigned int tiltDelay=300;
	unsigned int tilt_deg=100*tiltDelay;
	
	stepSpeed(&headMotor,headRPM);
	
	data='0';
	unsigned char data1;
	sei();
	
	while (1)
	{
		data1=data;
		if(UCSR0A&(1<<RXC0))
		{
			data = RX0_data();
		}
		
		switch(data){
			case 'a':		//스캐닝 동작
				stepSpeed(&headMotor,headRPM);
				TCCR0 = (1<<WGM01)|(0<<WGM00)|(0<<COM01)|(0<<COM00)|(1<<CS02)|(1<<CS01)|(1<<CS00);
				TIMSK |= (1<<OCIE0);
				OCR0 = 124;
				if(sonicCNT/125<=60)
				{
					hubMove(0,5000);
				}
				else if(sonicCNT/125>60)
				{
					TIMSK = (0<<OCIE0);
					sonicCNT=0;
					hubMove(0,0);
					isReady=2;
					data='e';
  				}
				
				if(sonicCNT/125>=15&&sonicCNT/125<35)
				{
					headRPM=40;
					if(headMotor.steps<headHeight*1.5)
					{
						timer2Token=1;
						stepperMove(&headMotor,1);
						if(headMotor.steps>headHeight*1.1)
						{
							if(tilt_deg/tiltDelay>75-20) tilt_deg--;
							else if(tilt_deg/tiltDelay<75-20) tilt_deg++;
						}
					}
					
					else if(headMotor.steps>=headHeight*1.5)
					{
						timer2Token=0;
						TIMSK &=(0<<OCIE2);
					}
				}
				if(sonicCNT/125>=35&&sonicCNT/125<52)
				{
					headRPM=20;
					if(headMotor.steps>headHeight*0.2)
					{
						timer2Token=1;
						stepperMove(&headMotor,-1);
						if(headMotor.steps>headHeight*0.6)
						{
							if(tilt_deg/tiltDelay>70-20) tilt_deg--;
							else if(tilt_deg/tiltDelay<70-20) tilt_deg++;
						}
					}
					else if(headMotor.steps<=headHeight*0.2)
					{
						timer2Token=0;
						TIMSK &=(0<<OCIE2);
						if(tilt_deg/tiltDelay>90-20) tilt_deg--;
						else if(tilt_deg/tiltDelay<90-20) tilt_deg++;
					}
				}
				if(sonicCNT/125>=52&&sonicCNT/125<60)
				{
					if(headMotor.steps>0)
					{
						timer2Token=1;
						stepperMove(&headMotor,-1);
						if(headMotor.steps<headHeight*0.1)
						{
							if(tilt_deg/tiltDelay>70-20) tilt_deg--;
							else if(tilt_deg/tiltDelay<70-20) tilt_deg++;
						}
					}
					else if(headMotor.steps<=0)
					{
						timer2Token=0;
						TIMSK &=(0<<OCIE2);
						headRPM=60;
					}
				}
				servoMoveTo(4,tilt_deg/tiltDelay);
				break;
			case 'r':	//헤드 이동 및 스캔 준비
				if(headMotor.steps<headHeight && headMotor.steps>=0)
					stepperMove(&headMotor,1);
				else if(headMotor.steps==headHeight)
				{
					if(timer2Token==1)
					{
						timer2Token=0;
						sensorInit();
						data='b';
					}
					if(isReady==1)
					{
						TX0_string("^r");
						isReady=0;
					}
				}
				else if(headMotor.steps>headHeight) stepperMove(&headMotor,-1);
				if(tilt_deg/tiltDelay>90-20) tilt_deg--;
				else if(tilt_deg/tiltDelay<90-20) tilt_deg++;
				servoMoveTo(4,tilt_deg/tiltDelay);
				break;
			case 'b':	//스캐닝 준비 초음파센서로 거리 조정
				turnWheel(90,90+1,90-3);
				
				//sensorInit();
				checkDist();
				//TX0_data(dist);
				//TIMSK = (0<<OCIE2);
				if(dist!=5){
					if(dist>setDist+2) hubMove(1,1000);
					else if(dist<setDist-2) hubMove(0,1000);
					else if(dist>=setDist-2 && dist<=setDist+2)
					{
						isReady=1;
						hubMove(0,0);
						turnWheel(servo1deg,servo2deg,servo3deg);
						_delay_ms(500);
						
						dist=0;
						data='r';
					}
				}
				break;
			case 'e':	//스캔 종료 후 원위치
				stepSpeed(&headMotor,60);
				if(headMotor.steps<headHeight && headMotor.steps>=0)
					stepperMove(&headMotor,1);
				else if(headMotor.steps==headHeight)
				{
					TIMSK = (0<<OCIE2);
					if(isReady==2)
					{
						TX0_string("^e");
						isReady=0;
					}
				}
				else if(headMotor.steps>headHeight) stepperMove(&headMotor,-1);
				if(tilt_deg/tiltDelay>90-20) tilt_deg--;
				else if(tilt_deg/tiltDelay<90-20) tilt_deg++;
				servoMoveTo(4,tilt_deg/tiltDelay);
				break;
			
			case '1':	//회전속도 ++
				if(hubSpeed<32000) hubSpeed+=1000;
				TX0_data(hubSpeed/1000);
				data=data1;
				break;
			case '2':	//회전속도 --
				if(hubSpeed>=1000) hubSpeed-=1000;
				TX0_data(hubSpeed/1000);
				data=data1;
				break;
				
			case '3':	//헤드 이동속도 ++
				stepSpeed(&headMotor,headRPM++);
				TX0_data(headRPM);
				data=data1;
				break;
			case '4':	//헤드 이동속도 --
				stepSpeed(&headMotor,headRPM--);
				TX0_data(headRPM);
				data= data1;
				break;
				
			case '5':	//헤드 위치 up
				stepSpeed(&headMotor,50);
				headHeight+=100;
				TX0_data(headHeight/100);
				data=data1;
				break;
			case '6':	//헤드 위치 down
				stepSpeed(&headMotor,50);
				headHeight-=100;
				TX0_data(headHeight/100);
				data=data1;
				break;
			case '7':	//거리 증가
				setDist+=5;
				TX0_data(setDist);
				data=data1;
			break;
			case '8':	//거리 감소
				setDist-=5;
				TX0_data(setDist);
				data= data1;
			break;
				
			case 'd':	//이동 모터 정방향(시계방향 회전 or 전진)
				hubMove(1,hubSpeed);
			break;
			case 'f':	//이동 모터 역방향(반시계방향 회전 or 후진)
				hubMove(0,hubSpeed);
			break;
			case 'w':
				stepSpeed(&headMotor,headRPM);
				if(headMotor.steps<headHeight)
				{
					timer2Token=1;
					stepperMove(&headMotor,1);
				}
				else if(headMotor.steps>=headHeight)
				{
					timer2Token=0;
					TIMSK &=(0<<OCIE2);
					PORTE |= (1<<PORTE5);
				}
				break;
			case 's':
				stepSpeed(&headMotor,headRPM);
				if(headMotor.steps>0)
				{
					timer2Token=1;
					stepperMove(&headMotor,-1);
				}
				else if(headMotor.steps<=0)
				{
					timer2Token=0;
					TIMSK &=(0<<OCIE2);
					PORTE |= (1<<PORTE5);
					data=0;
				}
				break;
			case 'j':
				turnWheel(0,160,20);
				break;
			case 'i':
				turnWheel(90,90-2,90-2);
				break;
			case 'o':
				servoMoveTo(2,servo2deg++);
				TX0_data(servo2deg);
				data=0;
				break;
			case 'p':
				servoMoveTo(2,servo2deg--);
				TX0_data(servo2deg);
				data=0;
				break;
			case 'k':
				servoMoveTo(3,servo3deg--);
				TX0_data(servo3deg);
				data=0;
				break;
			case 'l':
				servoMoveTo(3,servo3deg++);
				TX0_data(servo3deg);
				data=0;
				break;
				
			case 'n':
				heightEx = headMotor.steps;
				while (headMotor.steps-heightEx<heightMM/0.355+1)
				{
					stepperMove(&headMotor,1);
				}
				timer2Token=0;
				TIMSK &=(0<<OCIE2);
				data=0;
				break;
			case 'm':
				heightEx = headMotor.steps;
				while (heightEx-headMotor.steps<heightMM/0.355+1)
				{
					stepperMove(&headMotor,-1);
				}
				timer2Token=0;
				TIMSK &=(0<<OCIE2);
				data=0;
				break;
			case 'g':	//거리 증가
				heightMM+=10;
				TX0_data(heightMM/10);
				data=0;
				break;
			case 'h':	//거리 감소
				heightMM-=10;
				TX0_data(heightMM/10);
				data= 0;
				break;
				
			case 'c':		//시리얼 연결 후 call
				TX0_string("^c");
				data=0;
				break;
			case 'q':
				//sensorInit();
				checkDist();
				TX0_data(dist);
				break;
			default:
				timer2Token=0;
				TIMSK = (0<<OCIE2);
				sonicCNT=0;
				hubMove(0,0);
				servoMoveTo(1,servo1deg);
				servoMoveTo(2,servo2deg);
				servoMoveTo(3,servo3deg);
				break;
		}
	}
}