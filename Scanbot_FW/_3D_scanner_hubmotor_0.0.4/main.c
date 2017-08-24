#define F_CPU 16000000UL

//#define findHeight
//#define headspeed
//#define test
#define variableTest // 2100 - 1650,2100,1250,500,0 / 2250 - 1750,2250,1300,550,0 / 2400 - 1850,2400,1350,600,0

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>
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

ISR(TIMER2_COMP_vect) {		//motor2 & UltraSonic sensor count
	PORTE = PORTE^(timer2Token<<PINE5);
	headMotor.stepCnt += headMotor.dir*timer2Token;
	headMotor.steps = headMotor.stepCnt/2;
}
ISR(TIMER0_COMP_vect) {
	sonicCNT++;
}
ISR(INT6_vect) {		//UltraSonic rising edge

	TIMSK |= (1<<OCIE0);
	OCR0 = 19;
}
ISR(INT7_vect) {		//UltraSonic falling edge
	TIMSK &= (0<<OCIE0);
	dist = sonicCNT/6;
	sonicCNT = 0;
}

int main(void){
	cli();	
	stepperInit(&headMotor,2,2);
	//	sensorInit();
	servoInit();
	servoMoveTo(1,servo1deg);
	servoMoveTo(2,servo2deg);
	servoMoveTo(3,servo3deg);
	servoMoveTo(4,servo4deg);
	
	USART_Init(38400);
	sonicCNT=0;
	sonicFlag=0;
	dist=0;
	unsigned int setDist=80;
	//18 ~ 30 ~ 40 사이 값으로.
	//17부터 모터 최대값 잡힘.(고주파 잡음만)
	unsigned int headRPM=30;
	int hubSpeed=0;
	
	unsigned int headHeight=2000;
	unsigned int heightEx=0;
	unsigned int heightMM=100;
	
	unsigned int tiltDelay=300;
	unsigned int tilt_deg= 100*tiltDelay;

	unsigned int delta = 70;
	unsigned int deltaFlag = 1;
	unsigned int deltaCNT = 0;

	unsigned int H1 = 1650;
	unsigned int H2 = 2100;
	unsigned int H3 = 1250;
	unsigned int H4 = 500;
	
	stepSpeed(&headMotor,headRPM);
	
	data='0';
	unsigned char data1;
	sei();
	
	isReady=1;
	//TX0_string("^Ready");
	while (1)	{
		data1=data;
		if(UCSR0A&
		(1<<RXC0)){
			data = RX0_data();
		}
		/*
		CASE 1 1650 -> 2100
		CASE Q 1700 -> 2250
		CASE A 1700 -> 2400

		*/
		switch(data){
		#ifdef variableTest
			case 'q':
			H1 = 1650; H2 = 2100; H3 = 1250; H4 = 500;
			break;

			case 'w':
			H1 = 1750; H2 = 2250; H3 = 1300; H4 = 550;
			break;

			case 'e':
			H1 = 1850; H2 = 2400; H3 = 1350; H4 = 600;
			break;

			case '1':	//헤드 이동 및 스캔 준비
			headHeight = H1;
			servoMoveTo(4,70);
			if(headMotor.steps<headHeight && headMotor.steps>=0) {
				stepperMove(&headMotor,1);
			}
			else if(headMotor.steps==headHeight) {
				if(timer2Token==1) {
					timer2Token=0;
					data=0;
					isReady=1;
				}
				if(isReady==1) {
					TX0_string("#1");
					isReady=0;
				}
			}
			break;

			case '2':	//헤드 이동 및 스캔 준비
			headHeight = H2;
			if(headMotor.steps<headHeight && headMotor.steps>=0){
				stepperMove(&headMotor,1);
				//TX0_string("1");
				deltaCNT++;
				if(deltaCNT%10000 == 0 && delta > 54){
					delta--;
					servoMoveTo(4,delta);
				}
			}
			else if(headMotor.steps==headHeight){
				deltaCNT=0;
				if(timer2Token==1){
					timer2Token=0;
					data=0;
					isReady=1;
				}
				if(isReady==1) {
					deltaCNT=0;
					TX0_string("#2");
					isReady=0;
				}
			}
			else if(headMotor.steps>headHeight)
			stepperMove(&headMotor,-1);
			break;

			case '3':	//헤드 이동 및 스캔 준비
			headHeight = H3;
			if(headMotor.steps<headHeight && headMotor.steps>=0){
				stepperMove(&headMotor,-1);
			}
			else if(headMotor.steps==headHeight){
				if(timer2Token==1){
					timer2Token=0;
					data=0;
					isReady=1;
				}
				if(isReady==1){
					TX0_string("#3");
					delta = 70;
					isReady=0;
				}
			}
			else if(headMotor.steps>headHeight) {
				stepperMove(&headMotor,-1);
				deltaCNT++;
				if(deltaCNT%7200 == 0 && delta < 71){
					delta++;
					servoMoveTo(4,delta);
				}
			}
			break;

			case '4':	//헤드 이동 및 스캔 준비
			headHeight = H4;
			servoMoveTo(4,70); // init center
			if(headMotor.steps<headHeight && headMotor.steps>=0)
			stepperMove(&headMotor,-1);
			else if(headMotor.steps==headHeight){
				if(timer2Token==1){
					timer2Token=0;
					data=0;
					isReady=1;
				}
				if(isReady==1){
					TX0_string("#4");
					isReady=0;
				}
			}
			else if(headMotor.steps>headHeight)
			stepperMove(&headMotor,-1);
			break;

			case '5':	//헤드 이동 및 스캔 준비
			headHeight = 0;
			if(headMotor.steps<headHeight && headMotor.steps>=0) {
				stepperMove(&headMotor,-1);
			}
			
			else if(headMotor.steps==headHeight) {
				if(timer2Token==1){
					timer2Token=0;
					data=0;
					isReady=1;
				}
				if(isReady==1){
					TX0_string("#5");
					isReady=0;
				}
			}
			else if(headMotor.steps>headHeight)
			stepperMove(&headMotor,-1);
			break;
			
			case '7':
			delta = 50;
			servoMoveTo(4,delta);
			break;

			case '8':
			if(49 <= delta && delta <=71){
				servoMoveTo(4,delta);
				_delay_ms(120);
				delta--;
			}
			break;

			case '9':
			if(delta <=70){
				servoMoveTo(4,delta);
				_delay_ms(120);
				delta++;
			}
			break;

			case 'c':
			stepSpeed(&headMotor,headRPM);
			if(headMotor.steps>0){
				timer2Token=1;
				stepperMove(&headMotor,-1);
			}
			else if(headMotor.steps<=0){
				timer2Token=0;
				TIMSK &=(0<<OCIE2);
				PORTE |= (1<<PORTE5);
				data=0;
			}
			break;

		#endif

		#ifdef findHeight

			case '1':	//헤드 이동 및 스캔 준비
			headHeight = 800;
			//servoMoveTo(4,70);
			if(headMotor.steps<headHeight && headMotor.steps>=0) {
				stepperMove(&headMotor,1);
			}
			else if(headMotor.steps==headHeight) {
				if(timer2Token==1) {
					timer2Token=0;
					data=0;
					isReady=1;
				}
				if(isReady==1) {
					isReady=0;
				}
			}
			break;						
		#endif

		#ifdef headspeed
			case 'c':
			stepSpeed(&headMotor,headRPM);
			if(headMotor.steps>0){
				timer2Token=1;
				stepperMove(&headMotor,-1);
			}
			else if(headMotor.steps<=0){
				timer2Token=0;
				TIMSK &=(0<<OCIE2);
				PORTE |= (1<<PORTE5);
				data=0;
			}
			break;
			case 'q':	//헤드 이동속도 --
			stepSpeed(&headMotor,headRPM--);
			TX0_string("***");
			TX0_int(headRPM);
			data= data1;
			break;

			case 'w':	//헤드 이동속도 ++
			stepSpeed(&headMotor,headRPM++);
			TX0_int(headRPM);
			TX0_string("***");
			TX0_int(headRPM);
			data= data1;
			break;

			
			case '1':	//헤드 이동 및 스캔 준비
			headHeight = 800;
			servoMoveTo(4,70);
			if(headMotor.steps<headHeight && headMotor.steps>=0) {
				stepperMove(&headMotor,1);
			}
			else if(headMotor.steps==headHeight) {
				if(timer2Token==1) {
					timer2Token=0;
					data=0;
					isReady=1;
				}
				if(isReady==1) {
					
					//TX0_string("#1");
					isReady=0;
				}
			}
			break;

			case '2':	//헤드 이동 및 스캔 준비
			headHeight = 200;
			if(headMotor.steps<headHeight && headMotor.steps>=0){
				stepperMove(&headMotor,-1);
			}
			else if(headMotor.steps==headHeight){
				if(timer2Token==1){
					timer2Token=0;
					data=0;
					isReady=1;
				}
				if(isReady==1){
					//TX0_string("#3");
					delta = 70;
					isReady=0;
				}
			}
			else if(headMotor.steps>headHeight) {
				stepperMove(&headMotor,-1);
				deltaCNT++;
				if(deltaCNT%7200 == 0 && delta < 71){
					delta++;
					servoMoveTo(4,delta);
				}
			}
			break;

		#endif


		#ifdef test
			case 'q':
			headHeight = 2100;
			// 160
			break;

			case 'w':
			headHeight = 2250;
			//170
			break;

			case 'e':
			headHeight = 2400;
			//180
			break;

			case '1':	//헤드 이동 및 스캔 준비
			headHeight = 1650;
			servoMoveTo(4,70);
			if(headMotor.steps<headHeight && headMotor.steps>=0) {
				stepperMove(&headMotor,1);
			}
			else if(headMotor.steps==headHeight) {
				if(timer2Token==1) {
					timer2Token=0;
					data=0;
					isReady=1;
				}
				if(isReady==1) {
					TX0_string("#1");
					isReady=0;
				}
			}
			break;

			case '2':	//헤드 이동 및 스캔 준비
			if(headMotor.steps<headHeight && headMotor.steps>=0){
				stepperMove(&headMotor,1);
				//TX0_string("1");
				deltaCNT++;
				if(deltaCNT%10000 == 0 && delta > 54){
					delta--;
					servoMoveTo(4,delta);
				}
			}
			else if(headMotor.steps==headHeight){
				deltaCNT=0;
				if(timer2Token==1){
					timer2Token=0;
					data=0;
					isReady=1;
				}
				if(isReady==1) {
					deltaCNT=0;
					TX0_string("#2");
					isReady=0;
				}
			}
			else if(headMotor.steps>headHeight)
			stepperMove(&headMotor,-1);
			break;

			case '3':	//헤드 이동 및 스캔 준비
			headHeight = 1250;
			if(headMotor.steps<headHeight && headMotor.steps>=0){
				stepperMove(&headMotor,-1);
			}
			else if(headMotor.steps==headHeight){
				if(timer2Token==1){
					timer2Token=0;
					data=0;
					isReady=1;
				}
				if(isReady==1){
					TX0_string("#3");
					delta = 70;
					isReady=0;
				}
			}
			else if(headMotor.steps>headHeight) {
				stepperMove(&headMotor,-1);
				deltaCNT++;
				if(deltaCNT%7200 == 0 && delta < 71){
					delta++;
					servoMoveTo(4,delta);
				}
			}
			break;

			case '4':	//헤드 이동 및 스캔 준비
			headHeight = 500;
			servoMoveTo(4,70); // init center
			if(headMotor.steps<headHeight && headMotor.steps>=0)
			stepperMove(&headMotor,-1);
			else if(headMotor.steps==headHeight){
				if(timer2Token==1){
					timer2Token=0;
					data=0;
					isReady=1;
				}
				if(isReady==1){
					TX0_string("#4");
					isReady=0;
				}
			}
			else if(headMotor.steps>headHeight)
			stepperMove(&headMotor,-1);
			break;

			case '5':	//헤드 이동 및 스캔 준비
			headHeight = 0;
			if(headMotor.steps<headHeight && headMotor.steps>=0) {
				stepperMove(&headMotor,-1);
			}
			
			else if(headMotor.steps==headHeight) {
				if(timer2Token==1){
					timer2Token=0;
					data=0;
					isReady=1;
				}
				if(isReady==1){
					TX0_string("#5");
					isReady=0;
				}
			}
			else if(headMotor.steps>headHeight)
			stepperMove(&headMotor,-1);
			break;
			
			case '7':
			delta = 50;
			servoMoveTo(4,delta);
			break;

			case '8':
			if(49 <= delta && delta <=71){
				servoMoveTo(4,delta);
				_delay_ms(120);
				delta--;
			}
			break;

			case '9':
			if(delta <=70){
				servoMoveTo(4,delta);
				_delay_ms(120);
				delta++;
			}
			break;
			
			case '-':	//헤드 이동속도 --
			stepSpeed(&headMotor,headRPM--);
			TX0_data(headRPM);
			data= data1;
			break;

			case '+':	//헤드 이동속도 ++
			stepSpeed(&headMotor,headRPM++);
			TX0_data(headRPM);
			data= data1;
			break;

			case 'c':
			stepSpeed(&headMotor,headRPM);
			if(headMotor.steps>0){
				timer2Token=1;
				stepperMove(&headMotor,-1);
			}
			else if(headMotor.steps<=0){
				timer2Token=0;
				TIMSK &=(0<<OCIE2);
				PORTE |= (1<<PORTE5);
				data=0;
			}
			break;
			#endif
			//case 'a':		//스캐닝 동작
			//stepSpeed(&headMotor,headRPM);
			//TCCR0 = (1<<WGM01)|(0<<WGM00)|(0<<COM01)|(0<<COM00)|(1<<CS02)|(1<<CS01)|(1<<CS00);
			//TIMSK |= (1<<OCIE0);
			//OCR0 = 124;
			//if(sonicCNT/125<=60)
			//{
			//hubMove(0,5000);
			//}
			//else if(sonicCNT/125>60)
			//{
			//TIMSK = (0<<OCIE0);
			//sonicCNT=0;
			//hubMove(0,0);
			//isReady=2;
			//data='e';
			//}
			//
			//if(sonicCNT/125>=15&&sonicCNT/125<35)
			//{
			//headRPM=40;
			//if(headMotor.steps<headHeight*1.5)
			//{
			//timer2Token=1;
			//stepperMove(&headMotor,1);
			//if(headMotor.steps>headHeight*1.1)
			//{
			//if(tilt_deg/tiltDelay>75-20) tilt_deg--;
			//else if(tilt_deg/tiltDelay<75-20) tilt_deg++;
			//}
			//}
			//
			//else if(headMotor.steps>=headHeight*1.5)
			//{
			//timer2Token=0;
			//TIMSK &=(0<<OCIE2);
			//}
			//}
			//if(sonicCNT/125>=35&&sonicCNT/125<52)
			//{
			//headRPM=20;
			//if(headMotor.steps>headHeight*0.2)
			//{
			//timer2Token=1;
			//stepperMove(&headMotor,-1);
			//if(headMotor.steps>headHeight*0.6)
			//{
			//if(tilt_deg/tiltDelay>70-20) tilt_deg--;
			//else if(tilt_deg/tiltDelay<70-20) tilt_deg++;
			//}
			//}
			//else if(headMotor.steps<=headHeight*0.2)
			//{
			//timer2Token=0;
			//TIMSK &=(0<<OCIE2);
			//if(tilt_deg/tiltDelay>90-20) tilt_deg--;
			//else if(tilt_deg/tiltDelay<90-20) tilt_deg++;
			//}
			//}
			//if(sonicCNT/125>=52&&sonicCNT/125<60)
			//{
			//if(headMotor.steps>0)
			//{
			//timer2Token=1;
			//stepperMove(&headMotor,-1);
			//if(headMotor.steps<headHeight*0.1)
			//{
			//if(tilt_deg/tiltDelay>70-20) tilt_deg--;
			//else if(tilt_deg/tiltDelay<70-20) tilt_deg++;
			//}
			//}
			//else if(headMotor.steps<=0)
			//{
			//timer2Token=0;
			//TIMSK &=(0<<OCIE2);
			//headRPM=60;
			//}
			//}
			//servoMoveTo(4,tilt_deg/tiltDelay);
			//break;



			/////////////////
			/*
			case 'u':
			stepSpeed(&headMotor,50);
			headHeight+=100;
			if(headMotor.steps<headHeight && headMotor.steps>=0)
			stepperMove(&headMotor,1);
			else if(headMotor.steps==headHeight)
			{
			if(timer2Token ==1)
			{
			timer2Token=0;
			TIMSK &=(0<<OCIE2);
			PORTE |= (1<<PORTE5);
			data=0;
			}
			}
			//TX0_data(headHeight/100);
			data=data1;
			TX0_data(headMotor.steps);
			break;

			case 'd':
			stepSpeed(&headMotor,50);
			headHeight-=100;
			if(headMotor.steps<headHeight && headMotor.steps>=0)
			stepperMove(&headMotor,1);
			else if(headMotor.steps==headHeight)
			{
			if(timer2Token ==1)
			{
			timer2Token=0;
			}
			else if(headMotor.steps>headHeight)
			stepperMove(&headMotor,-1);
			}
			data=data1;
			break;

			///////////////////
			*/

			//case '5':	//헤드 위치 up
			//stepSpeed(&headMotor,50);
			//headHeight+=100;
			//TX0_data(headHeight/100);
			//data=data1;
			//break;
			//case '6':	//헤드 위치 down
			//stepSpeed(&headMotor,50);
			//headHeight-=100;
			//TX0_data(headHeight/100);
			//data=data1;
			//break;

			//case 'r':	//헤드 이동 및 스캔 준비
			//if(headMotor.steps<headHeight && headMotor.steps>=0)
			//stepperMove(&headMotor,1);
			//else if(headMotor.steps==headHeight)
			//{
			//if(timer2Token==1)
			//{
			//timer2Token=0;
			//sensorInit();
			//data='b';
			//}
			//if(isReady==1)
			//{
			//TX0_string("^r");
			//isReady=0;
			//}
			//}
			//else if(headMotor.steps>headHeight) stepperMove(&headMotor,-1);
			//if(tilt_deg/tiltDelay>90-20) tilt_deg--;
			//else if(tilt_deg/tiltDelay<90-20) tilt_deg++;
			//servoMoveTo(4,tilt_deg/tiltDelay);
			//break;
			//case 'b':	//스캐닝 준비 초음파센서로 거리 조정
			//turnWheel(90,90+1,90-3);
			//
			////sensorInit();
			//checkDist();
			////TX0_data(dist);
			////TIMSK = (0<<OCIE2);
			//if(dist!=5){
			//if(dist>setDist+2) hubMove(1,1000);
			//else if(dist<setDist-2) hubMove(0,1000);
			//else if(dist>=setDist-2 && dist<=setDist+2)
			//{
			//isReady=1;
			//hubMove(0,0);
			//turnWheel(servo1deg,servo2deg,servo3deg);
			//_delay_ms(500);
			//
			//dist=0;
			//data='r';
			//}
			//}
			//break;
			//case 'e':	//스캔 종료 후 원위치
			//stepSpeed(&headMotor,60);
			//if(headMotor.steps<headHeight && headMotor.steps>=0)
			//stepperMove(&headMotor,1);
			//else if(headMotor.steps==headHeight)
			//{
			//TIMSK = (0<<OCIE2);
			//if(isReady==2)
			//{
			//TX0_string("^e");
			//isReady=0;
			//}
			//}
			//else if(headMotor.steps>headHeight) stepperMove(&headMotor,-1);
			//if(tilt_deg/tiltDelay>90-20) tilt_deg--;
			//else if(tilt_deg/tiltDelay<90-20) tilt_deg++;
			//servoMoveTo(4,tilt_deg/tiltDelay);
			//break;
			//
			//case '1':	//회전속도 ++
			//if(hubSpeed<32000) hubSpeed+=1000;
			//TX0_data(hubSpeed/1000);
			//data=data1;
			//break;
			//case '2':	//회전속도 --
			//if(hubSpeed>=1000) hubSpeed-=1000;
			//TX0_data(hubSpeed/1000);
			//data=data1;
			//break;
			//
			//case '3':	//헤드 이동속도 ++
			//stepSpeed(&headMotor,headRPM++);
			//TX0_data(headRPM);
			//data=data1;
			//break;
			//case '4':	//헤드 이동속도 --
			//stepSpeed(&headMotor,headRPM--);
			//TX0_data(headRPM);
			//data= data1;
			//break;
			
			//case '7':	//거리 증가
			//setDist+=5;
			//TX0_data(setDist);
			//data=data1;
			//break;
			//case '8':	//거리 감소
			//setDist-=5;
			//TX0_data(setDist);
			//data= data1;
			//break;
			//
			//case 'd':	//이동 모터 정방향(시계방향 회전 or 전진)
			//hubMove(1,hubSpeed);
			//break;
			//case 'f':	//이동 모터 역방향(반시계방향 회전 or 후진)
			//hubMove(0,hubSpeed);
			//break;
			//case 'w':
			//stepSpeed(&headMotor,headRPM);
			//if(headMotor.steps<headHeight)
			//{
			//timer2Token=1;
			//stepperMove(&headMotor,1);
			//}
			//else if(headMotor.steps>=headHeight)
			//{
			//timer2Token=0;
			//TIMSK &=(0<<OCIE2);
			//PORTE |= (1<<PORTE5);
			//}
			//break;

			//case 'j':
			//turnWheel(0,160,20);
			//break;
			//case 'i':
			//turnWheel(90,90-2,90-2);
			//break;
			//case 'o':
			//servoMoveTo(2,servo2deg++);
			//TX0_data(servo2deg);
			//data=0;
			//break;
			//case 'p':
			//servoMoveTo(2,servo2deg--);
			//TX0_data(servo2deg);
			//data=0;
			//break;
			//case 'k':
			//servoMoveTo(3,servo3deg--);
			//TX0_data(servo3deg);
			//data=0;
			//break;
			//case 'l':
			//servoMoveTo(3,servo3deg++);
			//TX0_data(servo3deg);
			//data=0;
			//break;
			//
			//case 'n':
			//heightEx = headMotor.steps;
			//while (headMotor.steps-heightEx<heightMM/0.355+1)
			//{
			//stepperMove(&headMotor,1);
			//}
			//timer2Token=0;
			//TIMSK &=(0<<OCIE2);
			//data=0;
			//break;
			//case 'm':
			//heightEx = headMotor.steps;
			//while (heightEx-headMotor.steps<heightMM/0.355+1)
			//{
			//stepperMove(&headMotor,-1);
			//}
			//timer2Token=0;
			//TIMSK &=(0<<OCIE2);
			//data=0;
			//break;
			//case 'g':	//거리 증가
			//heightMM+=10;
			//TX0_data(heightMM/10);
			//data=0;
			//break;
			//case 'h':	//거리 감소
			//heightMM-=10;
			//TX0_data(heightMM/10);
			//data= 0;
			//break;
			//
			//case 'c':		//시리얼 연결 후 call
			//TX0_string("^c");
			//data=0;
			//break;
			//case 'q':
			////sensorinit();
			//checkdist();
			//tx0_data(dist);
			//break;
			default:
			//timer2Token=0;
			//timsk = (0<<ocie2);
			//TIMSK = (0<<OCIE2);
			//soniccnt=0;
			//hubmove(0,0);
			//servomoveto(1,servo1deg);
			//servomoveto(2,servo2deg);
			//servomoveto(3,servo3deg);
			break;
		}
	}
}