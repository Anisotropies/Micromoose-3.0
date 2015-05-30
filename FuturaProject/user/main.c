#include "stm32f4xx.h"
#include "delay.h"
#include "led.h"
#include "button.h"
#include <stdio.h>
#include "usart.h"
#include "SPI.h"
#include "matrixDisplay.h"
#include "pwm.h"
#include "encoder.h"
#include "buzzer.h"
#include "main.h"
#include "sensor_Function.h"
#include "adc.h"
const int RIGHT = -1;
const int LEFT = 1;

/* * * * * * * * * *
 * Setup Variables *
 * * * * * * * * * */
//Current Speed Variables
float curSpeedX = 0; //Current Translational Speed
float curSpeedW = 0; //Current Rotational Speed
//Target Speed Variables
int targetSpeedX = 0; //Target Translational Speed
int targetSpeedW = 0; //Target Rotational Speed
//Encoder Feedback Variables
int encoderFeedbackX = 0; //Translational Encoder Feedback
int encoderFeedbackW = 0; //Rotational Encoder Feedback
//Encoder Variables
int leftEncoder; 
int rightEncoder;
int leftEncoderChange;
int rightEncoderChange;
int encoderChange;
int leftEncoderOld;
int rightEncoderOld;
int leftEncoderCount;
int rightEncoderCount;
int encoderCount;
int oldEncoderCount;
int distanceLeft;
//Error Variables
float posErrorX = 0;
float posErrorW = 0;
float oldPosErrorX = 0;
float oldPosErrorW = 0;
//PWM Variables
float posPwmX = 0;
float posPwmW = 0;
int32_t hasLeftWall = 615;
int32_t hasRightWall = 815;  
int32_t hasFrontWallLeft = 400; 
int32_t hasFrontWallRight = 400; 
/* * * * * * * * * * * * *
 * Adjustable Variables  *
 * * * * * * * * * * * * */

//IR Thresholds
int LFValue1 = 1420;
int RFValue1 = 1280;
int LFValue2 = 480;
int RFValue2 = 420;
int DLMiddleValue = 1048;
int DRMiddleValue = 1280;
//IR Sensor Calibration
int sensorError = 0;

//Feedback Control Variables
bool onlyUseGyroFeedback = false;
bool onlyUseEncoderFeedback = false;
//Sensor Variables
bool b_useIR = true;
bool b_useGyro = true;
bool b_useSpeedProfile = true;
//PID Variables
float pidInputX = 0;
float pidInputW = 0;
float kpX = .05, kdX = 0;
float kpW = .05, kdW = 0;
float kpW1 = 1;
float kdW1 = 26;
float kpW2 = 1;
float kdW2 = 36;
//Acceleration Variables
//temp
float accTrans = 2;
float decTrans = 2;
float accX = 600;
float decX = 600;
float accW = 1;
float decW = 1;

//Distance Variables
int oneCellDistance = 4500;
int partialCellDistance = 5500;

//Speed Settings
/*
int moveSpeed = speed_to_counts(500*2);
int turnSpeed = speed_to_counts(500*2);
int returnSpeed = speed_to_counts(500*2);
int stopSpeed = speed_to_counts(100*2);
int maxSpeed = speed_to_counts(2000*2);
*/
//Gyro Variables
int gyroFeedbackRatio = 5700;
int curAng;
//Analog Scale (?)
int a_scale = 1;



/* * * * * * * * * * * * * 
 * Function Declarations *
 * * * * * * * * * * * * */
void getSensorError(void);
void speedProfile(void);
void getEncoderStatus(void);
void calculateMotorPwm(void);
void updateCurrentSpeed(void);
int speed_to_counts(int accel);


/* * * * * * * * * *
 * Systick Handler *
 * * * * * * * * * */
void systick()
{
	if (b_useIR)
	{
		readSensor();
	}
	if (b_useGyro)
	{
		readGyro();
	}
	if (b_useSpeedProfile)
	{
		speedProfile();
	}
}

/* * * * * * * * * * * * *
 * Caculate Sensor Error *
 * * * * * * * * * * * * */
void getSensorError(void)//the very basic case
{
	if(DLSensor > DLMiddleValue && DRSensor < DRMiddleValue)
		sensorError = DLMiddleValue - DLSensor;
	else if(DRSensor > DRMiddleValue && DLSensor < DLMiddleValue)
		sensorError = DRSensor - DRMiddleValue;
	else
		sensorError = 0;
}

/* * * * * * * * * * * * * * * * * * * *
 *            Speed Profile            *
 *     Calculate what current speed    *
 * for each wheel should be and set it *
 * * * * * * * * * * * * * * * * * * * */
void speedProfile()
{
	getEncoderStatus();
	updateCurrentSpeed();
	calculateMotorPwm();
}

/* * * * * * * * * * * * * * * * * * * * *
 *           Get Encoder Status          *
 *  Determine how much further to travel *
 * * * * * * * * * * * * * * * * * * * * */

void getEncoderStatus()
{
	//displayMatrix("Enc");
	leftEncoder = getLeftEncCount();
	rightEncoder = getRightEncCount();

	leftEncoderChange = leftEncoder - leftEncoderOld;
	rightEncoderChange = rightEncoder - rightEncoderOld;
	encoderChange = (leftEncoderChange + rightEncoderChange)/2;
	leftEncoderOld = leftEncoder;
	rightEncoderOld = rightEncoder;

	leftEncoderCount += leftEncoderChange;
	rightEncoderCount += rightEncoderChange;
	encoderCount = (leftEncoderCount + rightEncoderCount)/2;

	distanceLeft -= encoderChange;
}

/* * * * * * * * * * * * * * * * * * * * *
 *           Update Current Speed        *
 *  Determine how much further to travel *
 * * * * * * * * * * * * * * * * * * * * */

void updateCurrentSpeed(void)
{
	//displayMatrix("Upd");
	//displayInt(curSpeedX);
	if(curSpeedX < targetSpeedX)
	{
		//displayMatrix("sped");
		curSpeedX += accTrans;
		if(curSpeedX > targetSpeedX)
			curSpeedX = targetSpeedX;
	}
	else if(curSpeedX > targetSpeedX)
	{
		//displayMatrix("slow");
		curSpeedX -= decTrans;
		if(curSpeedX < targetSpeedX)
			curSpeedX = targetSpeedX;
	}
	if(curSpeedW < targetSpeedW)
	{
		curSpeedW += accW;
		if(curSpeedW > targetSpeedW)
			curSpeedW = targetSpeedW;
	}
	else if(curSpeedW > targetSpeedW)
	{
		curSpeedW -= decW;
		if(curSpeedW < targetSpeedW)
			curSpeedW = targetSpeedW;
	}	
	//printf("curSpeedX %f targetSpeedX %d \r\n", curSpeedX, targetSpeedX);
}
/* * * * * * * * * * * * * * * *
 *      Calculate Motor PWM    *
 *   Take speed variables and  *
 *  calculate motor pwm values *
 * * * * * * * * * * * * * * * */
void calculateMotorPwm()
{
 	int gyroFeedback;
 	int rotationalFeedback;
 	int sensorFeedback;
 	int leftBaseSpeed;
 	int rightBaseSpeed;
	//displayMatrix("1");//displayMatrix("PWM");
 	encoderFeedbackX = rightEncoderChange + leftEncoderChange;
 	encoderFeedbackW = rightEncoderChange - leftEncoderChange;
	//displayInt(encoderFeedbackX);
 	gyroFeedback = aSpeed/gyroFeedbackRatio; 
 	sensorFeedback = sensorError/a_scale; //what is a_scale?

 	if (onlyUseGyroFeedback)
	{
 		rotationalFeedback = gyroFeedback;
	}
 	else if (onlyUseEncoderFeedback)
	{
 		rotationalFeedback = encoderFeedbackW;
	}
 	else
 		rotationalFeedback = encoderFeedbackW + gyroFeedback;
	//	displayInt(sensorFeedback);
	//rotationalFeedback = sensorFeedback;
	//printf("Encoder FeedbackX = %d\r\n", encoderFeedbackX);
 	posErrorX += curSpeedX -encoderFeedbackX;
 	posErrorW += curSpeedW - rotationalFeedback;
	//displayMatrix("2");
 	posPwmX = (kpX * posErrorX) + (kdX * (posErrorX - oldPosErrorX));
 	posPwmW = (kpW * posErrorW) + (kdW * (posErrorW - oldPosErrorW));
	
 	oldPosErrorX = posErrorX;
 	oldPosErrorW = posErrorW;
 	leftBaseSpeed = posPwmX - posPwmW;
 	rightBaseSpeed = posPwmX + posPwmW;
 	setLeftPwm(leftBaseSpeed);
 	setRightPwm(rightBaseSpeed);
}
int speed_to_counts(int accel)
{
		return accel;
}

int abs(int num)
{
	if (num < 0)
	{
		num = -num;
	}
	return num;
}

int needToDecelerate(int32_t dist, int16_t curSpd,  int16_t endSpd)
{
	if (curSpd < 0)
		curSpd = -curSpd;
	if (endSpd < 0)
		endSpd  = -endSpd;
	if (dist < 0)
		dist = 1;
	if (dist == 0)
		dist = 1;
	return (abs((curSpd*curSpd - endSpd*endSpd)*100/dist/4/2));
}



/* * * * * * * * * * * * * * * * * * * *
 * Basic Movements										 *
 * 1. forwardDistance								   *
 * 2. moveOneCell			                 *
 * * * * * * * * * * * * * * * * * * * */

void forwardDistance(int distance, int left_speed, int right_speed, bool coast) 
{
	distanceLeft = distance;
	while (distanceLeft>=0) 
	{
		targetSpeedX = 10;
		targetSpeedW = 0;
	}
	  targetSpeedX = 0;
		targetSpeedW = 0;
}
void moveOneCell(int moveSpeed, int maxSpeed)
{
	targetSpeedX = moveSpeed;
	targetSpeedW = 0;
	
	do
	{
		if (needToDecelerate(distanceLeft, curSpeedX, moveSpeed) < decX)
		{
			targetSpeedX = maxSpeed;
		}
		else
		{
			targetSpeedX = moveSpeed;
		}
	} while(((encoderCount - oldEncoderCount) < oneCellDistance && LFSensor < LFValue2 && RFSensor < RFValue2) 
	|| (LFSensor < LFValue1 && LFSensor > LFValue2) 
	|| (RFSensor < RFValue1 && RFSensor > RFValue2));
	
	oldEncoderCount = encoderCount;
		targetSpeedX = 0;
	targetSpeedW = 0;
}
void movePartialCell(int moveSpeed, int maxSpeed)
{
	targetSpeedX = moveSpeed;
	targetSpeedW = 0;
	
	do
	{
		if (needToDecelerate(distanceLeft, curSpeedX, moveSpeed) < decX)
		{
			targetSpeedX = maxSpeed;
		}
		else
		{
			targetSpeedX = moveSpeed;
		}
	} while(((encoderCount - oldEncoderCount) < partialCellDistance && LFSensor < LFValue2 && RFSensor < RFValue2) 
	|| (LFSensor < LFValue1 && LFSensor > LFValue2) 
	|| (RFSensor < RFValue1 && RFSensor > RFValue2));
	
	oldEncoderCount = encoderCount;
		targetSpeedX = 0;
	targetSpeedW = 0;
}

void turnRightAngle(int direction)
{
	curAng = angle;
	while (abs(angle-curAng) < abs(10000)) {
			//printf("angle: %d\tcurAngle: %d\tangle-curAngle: %d\r\n", angle, curAng, angle-curAng); 
			targetSpeedW = direction*5;
	}
	targetSpeedX = 0;
	targetSpeedW = 0;
}

void button1_interrupt(void) 
{
}

void button2_interrupt(void) 
{
}

int main(void) 
{
	int i;
	Systick_Configuration();
	LED_Configuration();
	button_Configuration();
	usart1_Configuration(9600);
	SPI_Configuration();
  TIM4_PWM_Init();
	Encoder_Configration();
	buzzer_Configuration();
	ADC_Config();


while(1) {
	targetSpeedX = 0;
	targetSpeedW = 0;
delay_ms(1000);
	 forwardDistance(1500,0,0,true);
		delay_ms(1000);
      if((DRSensor < hasRightWall)) {
        //Go Right
        displayMatrix("RIGT");
        movePartialCell(10,20);
				delay_ms(1000);
        turnRightAngle(RIGHT);
				delay_ms(1000);
        forwardDistance(400,0,0,true);
				delay_ms(1000);
	
      } else if((LFSensor < hasFrontWallLeft) && RFSensor < hasFrontWallRight) 
				{
        //Go Foward one cell
					moveOneCell(10, 20);
					delay_ms(1000);
        //forwardDistance(4000,0,0,true);
					displayMatrix("FWD");
					delay_ms(1000);
				} 
			else 
				{
        //Turn Left
				displayMatrix("LEFT");
        movePartialCell(10,20);
					delay_ms(1000);
        turnRightAngle(LEFT);
					delay_ms(1000);
        forwardDistance(400,0,0,true);

					delay_ms(1000);
      }
    }
			
	return 0;
}
