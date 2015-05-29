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
#include <string.h>
#include <cstring>


 ///////////////////////////
//Floodfill//////////////////
///////////////////////////

enum Dir {
    NORTH = 0,
    SOUTH,
    EAST,
    WEST,
    INVALID
};

struct Coord {
 
    int m_distance;
    int m_xPos;
    int m_yPos;
    bool m_isProcessed;
  
};


///////////////////////////
//Globals//////////////////
///////////////////////////


//CONSTANTS
int32_t hasLeftWall = 325;
int32_t hasRightWall = 714;  
int32_t hasFrontWallLeft = 400; 
int32_t hasFrontWallRight = 400; 
int32_t leftMiddleValue = 0;//1765;
int32_t rightMiddleValue = 0;//98;

const int xGOAL = 7;
const int yGOAL = 7;

double P = 0.01;
double D = 0.09;
int leftBaseSpeed = 100;
int rightBaseSpeed = 100;
volatile int left_enc;
volatile int right_enc;
volatile int cur_angle;
int forward_left_pwm=20;
int forward_right_pwm=20;

//NOT CONSTANTS
int32_t errorP = 0;
int32_t errorD = 0;
double totalError = 0;
int32_t oldErrorP = 0;

//ENCODER CONSTANTS VARIABLES
int rightEncoderDeltaCell = 5000;
int leftEncoderDeltaCell = 4500;
int leftEncoderDeltaCellInit = 2000;
int targetLeft = 0;
int targetRight = 0;
const int LEFT = 1;
const int RIGHT = -1;
int turning = 0;
int mouseStarted = 0;
int buttonPressed = 1;
char xCor;
char yCor;
char *dispBuf;

///////////////////////////
/////Floodfill//////////////////////
///////////////////////////
bool visitedStart = false;
enum Dir heading;
bool shouldGoForward;
bool shouldGoToCenter = true;
int x = 0; 
int y = 1;

const int MAZE_W = 16;
const int MAZE_L = 16;
 
struct Coord c_array[16][16]; //Contains Manhattan distances for each cell and if visited or not

const int MoveForward = 0;
const int TurnClockwise = 1;
const int TurnCounterClockwise = 2;
const int TurnAround = 3; 
const unsigned VECTOR_SIZE = 16;

bool frontWall = false;
bool leftWall = true;
bool rightWall = true;
         
void stop(int);

uint16_t vectorH[(VECTOR_SIZE * VECTOR_SIZE) / (8*sizeof(uint16_t))]; //Horizontal
uint16_t vectorV[(VECTOR_SIZE * VECTOR_SIZE) / (8*sizeof(uint16_t))]; //Vertical

void setH(unsigned x, unsigned y) {
   if(x < VECTOR_SIZE && y < VECTOR_SIZE)
				vectorH[x] |= 1<<y;
}
void clearH(unsigned x, unsigned y) {
   if(x < VECTOR_SIZE && y < VECTOR_SIZE)
       vectorH[x] &= ~(1<<y);
   }
bool getH(unsigned x, unsigned y) {
   if(x < VECTOR_SIZE && y < VECTOR_SIZE)
        return (vectorH[x] & 1<<y) != 0;
 
        return 0;
}
 
void clearAllH() {
    memset(vectorH, 0, sizeof(vectorH));
}
 
void setAllH() {
		memset(vectorH, ~0, sizeof(vectorH));
}

void setV(unsigned x, unsigned y) {
   if(x < VECTOR_SIZE && y < VECTOR_SIZE)
				vectorV[x] |= 1<<y;
}
 
void clearV(unsigned x, unsigned y) {
   if(x < VECTOR_SIZE && y < VECTOR_SIZE)
       vectorV[x] &= ~(1<<y);
   }
 
bool getV(unsigned x, unsigned y) {
   if(x < VECTOR_SIZE && y < VECTOR_SIZE)
        return (vectorV[x] & 1<<y) != 0;
 
        return 0;
}
 
void clearAllV() {
    memset(vectorV, 0, sizeof(vectorH));
}
 
void setAllV() {
		memset(vectorV, ~0, sizeof(vectorH));
}


struct stack {
   struct Coord s[300];
   int top;
} st;
 
int stfull() {
   if (st.top >= 300 - 1)
      return 1;
   else
      return 0;
}
 
void push(struct Coord item) {
   st.top++;
   st.s[st.top] = item;
}
 
int stempty() {
   if (st.top <= 0)
      return 1;
   else
      return 0;
}
 
struct Coord pop() {
   struct Coord item;
   item = st.s[st.top];
   st.top--;
   return (item);
}
 
enum Dir opposite(enum Dir d) {
    switch (d) {
        case NORTH:
            return SOUTH;
        case SOUTH:
            return NORTH;
        case EAST:
            return WEST;
        case WEST:
            return EAST;
        case INVALID:
        default:
            return INVALID;
    }
}
 
enum Dir clockwise(enum Dir d) {
    switch (d) {
        case NORTH:
            return EAST;
        case SOUTH:
            return WEST;
        case EAST:
            return SOUTH;
        case WEST:
            return NORTH;
        case INVALID:
        default:
            return INVALID;
    }
}
 
enum Dir counterClockwise(enum Dir d) {
    switch (d) {
        case NORTH:
            return WEST;
        case SOUTH:
            return EAST;
        case EAST:
            return NORTH;
        case WEST:
            return SOUTH;
        case INVALID:
        default:
            return INVALID;
    }
}



void calculateManahattan(int goalX, int goalY) {
	//set Manhattan Distances
        int xMid = goalX;
        int yMid = goalY;
	
				int i = 0;
				int j = 0;
         
        //Quadrant 1
        for (i = 0; i < 8; i++) {
            for (j = 0; j < 8; j++) {
                c_array[i][j].m_distance = abs(xMid - i) + abs(yMid - j);
            }
        }
        //Quadrant 2
        for (i = 8; i < 16; i++) {
            for (j = 0; j < 8; j++) {
                c_array[i][j].m_distance = abs(xMid - i) - 1 + abs(yMid - j);
            }
        }
        //Quadrant 3
        for (i = 8; i < 16; i++) {
            for (j = 8; j < 16; j++) {
                c_array[i][j].m_distance = abs(xMid - i) + abs(yMid - j) - 2;
            }
        }
         
        //Quadrant 4
        for (i = 0; i < 8; i++) {
            for (j = 8; j < 16; j++) {
                c_array[i][j].m_distance = abs(xMid - i) + abs(yMid - j) - 1;
            }
        }
         
        for (i = 0; i < 16; i++) {
            for (j = 0; j < 16; j++) {
                //std::cout << c_array[i][j].m_distance;
                //std::cout << " ";
            }
            //std::cout << "" << std::endl;
        }
        for (i = 0; i < 16; i++)
        {
            for (j = 0; j < 16; j++)
            {
                c_array[i][j].m_xPos = i;
                c_array[i][j].m_yPos = j;
            }
        }
    }

bool isWallBetween(struct Coord pos1, struct Coord pos2)
{
	if (pos1.m_xPos == pos2.m_xPos)
  {
		if (pos1.m_yPos > pos2.m_yPos)
		{
			return getH(pos2.m_xPos, pos2.m_yPos);
    }
		else if (pos1.m_yPos < pos2.m_yPos)
    {
			return getH(pos1.m_xPos, pos1.m_yPos);
    }
		else return false;
	}
  else if (pos2.m_yPos == pos1.m_yPos)
  {
		if (pos1.m_xPos > pos2.m_xPos)
		{
			return getV(pos2.m_xPos, pos2.m_yPos);
		}
		else if (pos1.m_xPos < pos2.m_xPos)
		{
			return getV(pos1.m_xPos, pos1.m_yPos);
		}
		else return false;
	}
	return false;
}

void floodFill(int xPos, int yPos) {
	
		//struct stack StackTemp;
		
		int i = 0;
		int shortest = 500;
		//shortBeep(500, 1000);
		printf("Running FloodFill!\r\n");
	
		push(c_array[xPos][yPos]);
		while (!stempty())
		{
				struct Coord cur = pop();
			
				cur.m_isProcessed = true;
				if (cur.m_distance == 0)
				{
						continue;
				}
				shortest = 500;
				for (i = 0; i < 4; i++)
				{
						struct Coord neighbor;
						//Set neighbor to the currect coordinate
						switch (i)
						{
								case NORTH:
										if (cur.m_yPos >= 15)
												continue;
										neighbor = c_array[cur.m_xPos][cur.m_yPos + 1];
										break;
								case SOUTH:
										if (cur.m_yPos <= 0)
												continue;
										neighbor = c_array[cur.m_xPos][cur.m_yPos - 1];
										break;
								case EAST:
										if (cur.m_xPos >= 15)
												continue;
										neighbor = c_array[cur.m_xPos + 1][cur.m_yPos];
										break;
								case WEST:
										if (cur.m_xPos <= 0)
												continue;
										neighbor = c_array[cur.m_xPos - 1][cur.m_yPos];
										break;
						}
						//Check if there is a wall between cur and neighbor
						if (!isWallBetween(cur, neighbor))
						{
								if (neighbor.m_distance < shortest)
								{
										shortest = neighbor.m_distance;
								}
								if (!neighbor.m_isProcessed)
								{
										neighbor.m_isProcessed = true;
								}
						}
				}
				if (shortest == 500)
				{
						continue;
				}
				if (cur.m_distance == (shortest + 1))
				{
						continue;
				}
				else
				{
						c_array[cur.m_xPos][cur.m_yPos].m_distance = shortest + 1;
				}
				for (i = 0; i < 4; i++)
				{
						struct Coord neighbor;
						//Set neighbor to the currect coordinate
						switch (i)
						{
								case NORTH: 
										if (cur.m_yPos >= 15)
												continue;
										neighbor = c_array[cur.m_xPos][cur.m_yPos + 1];
										break;
								case SOUTH:
										if (cur.m_yPos <= 0)
												continue;
										neighbor = c_array[cur.m_xPos][cur.m_yPos - 1];
										break;
								case EAST:
										if (cur.m_xPos >= 15)
												continue;
										neighbor = c_array[cur.m_xPos + 1][cur.m_yPos];
										break;
								case WEST:
										if (cur.m_xPos <= 0)
												continue;
										neighbor = c_array[cur.m_xPos - 1][cur.m_yPos];
										break;
						}
						//Check if there is a wall between cur and neighbor
						if (!isWallBetween(cur, neighbor))
						{
								push(neighbor);
						}
				}
		}
}

		
int nextMovement() {

		int i = 0;
		int j = 0;
	
		int frontX = 0;
		int frontY = 0;
		int rightX = 0;
		int rightY = 0;
		int leftX = 0;
		int leftY = 0;
		 
		// If we hit the start of the maze a second time, then
		// we couldn't find the center and never will...
		if (x == 0 && y == 0) {
				if (visitedStart) {
//                std::cout << "Time to go back to the center" << std::endl;
//                calculateManahattan(0, 0);
//                floodFill(x, y);
//                heading = opposite(heading);
//                return TurnAround;
				}
				else {
						visitedStart = true;
				}
		}
		 
		 
		//Set wall positions
		//LEFTWALL
		if (leftWall) {
				if (heading == NORTH) {
						if (x != 0){
								setV(x - 1, y);
						}
				}
				else if (heading == SOUTH) {
						setV(x, y);
						 
				}
				else if (heading == WEST) {
						if (y != 0){
								setH(x, y - 1);
						}
						 
				}
				else if (heading == EAST) {
						setH(x, y);
				}
		}
		 
		//RIGHTWALL
		if (rightWall) {
				if (heading == NORTH) {
						setV(x, y);
						 
				}
				else if (heading == SOUTH) {
						if (x != 0){
								setV(x - 1, y);
						}
						 
				}
				else if (heading == WEST) {
						setH(x, y);
						 
				}
				else if (heading == EAST) {
						if (y != 0){
								setH(x, y - 1);
						}
						 
				}
				 
		}
		//FRONTWALL
		if (frontWall)
		{
				if (heading == NORTH)
				{
						setH(x, y);
				}
				else if (heading == SOUTH)
				{
						if (y != 0)
						{
								setH(x, y - 1);
						}
				}
				else if (heading == WEST)
				{
						if (x != 0)
						{
								setV(x - 1, y);
						}
						 
				}
				else if (heading == EAST)
				{
						setV(x, y);
				}
		}
		 

		 
		switch (heading)
		{
				case NORTH:
						frontX = x;
						frontY = y + 1;
						rightX = x + 1;
						rightY = y;
						leftX = x - 1;
						leftY = y;
						break;
				case SOUTH:
						frontX = x;
						frontY = y - 1;
						rightX = x - 1;
						rightY = y;
						leftX = x + 1;
						leftY = y;
						break;
				case EAST:
						frontX = x + 1;
						frontY = y;
						rightX = x;
						rightY = y - 1;
						leftX = x;
						leftY = y + 1;
						break;
				case WEST:
						frontX = x - 1;
						frontY = y;
						rightX = x;
						rightY = y + 1;
						leftX = x;
						leftY = y - 1;
						break;
				default:
						break;
		}
		
		printf("Current position: ");
		if (isWallBetween(c_array[x][y], c_array[leftX][leftY])) {
			printf("Left wall\t");
		}
		else {
			printf("NO left wall\t");
		}
		if (isWallBetween(c_array[x][y], c_array[rightX][rightY])) {
			printf("Right Wall\t");
		}
		else {
			printf("NO right wall\t");
		}
		if (isWallBetween(c_array[x][y], c_array[frontX][frontY]))
		{
			printf("Front Wall\r\n");
		}
		else {
			printf("NO front wall\r\n");
		}
		 
		 
		//if (!frontWall) {
		printf("Current Coordiantes: (%d, %d)\r\n", x, y);
		printf("Current Manhattan: %d\r\n", c_array[x][y].m_distance);
		if (((frontX <= 15 && frontY <= 15) && (frontX >= 0 && frontY > 0)) && !(isWallBetween(c_array[x][y], c_array[frontX][frontY])) && c_array[x][y].m_distance > c_array[frontX][frontY].m_distance)
		{
			//printf("Moving forward\r\n");
				return MoveForward;
		}
		else if (((rightX <= 15 && rightY <= 15) && (rightX >= 0 && rightY > 0)) && !(isWallBetween(c_array[x][y], c_array[rightX][rightY])) && c_array[x][y].m_distance > c_array[rightX][rightY].m_distance)
		{
			//printf("Turning right\r\n");
			heading = clockwise(heading);
			return TurnClockwise;	
		}
		else if (((leftX <= 15 && leftY <= 15) && (leftX >= 0 && leftY > 0)) && !(isWallBetween(c_array[x][y], c_array[leftX][leftY])) && c_array[x][y].m_distance > c_array[leftX][leftY].m_distance)
		{
			//printf("Turning left\r\n");
			//shortBeep(250, 5000);
			heading = counterClockwise(heading);
			return TurnCounterClockwise;
		}
		 
		else if (isWallBetween(c_array[x][y], c_array[frontX][frontY]) && isWallBetween(c_array[x][y], c_array[rightX][rightY]) && isWallBetween(c_array[x][y], c_array[leftX][leftY]))
		{
				//printf("Turning around\r\n");
				heading = opposite(heading);
				return TurnAround;
		}
		 

		 
		//need to edit more for corner cases
		if (c_array[x][y].m_distance != 0)
		{
			
			if ((heading == NORTH && c_array[x][y].m_distance > c_array[x][y-1].m_distance && !isWallBetween(c_array[x][y], c_array[x][y-1])) || (heading == SOUTH && c_array[x][y].m_distance > c_array[x][y+1].m_distance && !isWallBetween(c_array[x][y], c_array[x][y+1])) || (heading == EAST && c_array[x][y].m_distance > c_array[x-1][y].m_distance && !isWallBetween(c_array[x][y], c_array[x-1][y])) || (heading == WEST && c_array[x][y].m_distance > c_array[x+1][y].m_distance && !isWallBetween(c_array[x][y], c_array[x+1][y])))
			{
				heading = opposite(heading);
				return TurnAround;
			}
		
			//std::cout << "Running Floodfill" << std::endl;
			displayMatrix("FILL");
			floodFill(x, y);
			
			return nextMovement();
		}
		else
		{
				shouldGoToCenter = !shouldGoToCenter;
				if (shouldGoToCenter)
				{
						//std::cout << "Went back to the beginning. Now to go back to the center" << std::endl;
						calculateManahattan(xGOAL, yGOAL);
						 
						//Call Floodfill on every cell
						for (i = 0; i < 16; i++)
						{
								for (j = 0; j < 16; j++)
								{
										floodFill(i, j);
								}
						}
						heading = opposite(heading);
						return TurnAround;
				}
				else
				{
						//std::cout << "Found center! Now to go back to the beginning" << std::endl;
					displayMatrix("END!");
					shortBeep(2000, 8000);
					while (1) {
						stop(100);
					}
					delay_ms(5000);
						calculateManahattan(0, 0);
						floodFill(x, y);
						heading = opposite(heading);
						return TurnAround;
				}
		}
}




///////////////////////////
//Basic movements//////////
///////////////////////////

void PID(void) 
{

	if((DLSensor > hasLeftWall && DRSensor > hasRightWall))//has both walls
	{
	//	printf("Both\r\n");
	//ccw direction is positive
		errorP = DRSensor - DLSensor - (rightMiddleValue - leftMiddleValue);
		//rightMiddleValue - leftMiddleValue is the offset between left and right sensor when mouse in the middle of cell
		errorD = errorP - oldErrorP;
		//printf("%d", errorP);
	}        
	else if((DLSensor > hasLeftWall))//only has left wall
	{
	//	printf("L\r\n");
		errorP = 2 * (leftMiddleValue - DLSensor);
		errorD = errorP - oldErrorP;
	}
	else if((DRSensor > hasRightWall))//only has right wall
	{
		//printf("R\r\n");
		errorP = 2 * (DRSensor - rightMiddleValue);
		errorD = errorP - oldErrorP;
	}

	else if((DLSensor < hasLeftWall && DRSensor <hasRightWall))//no wall, use encoder or gyro
	{
	//printf("None\r\n");
		errorP = 0;//(leftEncoder – rightEncoder*1005/1000)*3;
		errorD = 0;
	}
	forward_left_pwm = targetLeft;
	forward_right_pwm = targetRight;
	if (turning == 0) {
		totalError = P * errorP + D * errorD;
		oldErrorP = errorP;
		forward_left_pwm -= totalError;
		forward_right_pwm += totalError;
	}
	setLeftPwm(forward_left_pwm);
	setRightPwm(forward_right_pwm);
		/*MIGHT USE FOR STOPPPING
	if(LFSensor >= hasFrontWallLeft || RFSensor >= hasFrontWallRight)
	{
		forward_left_pwm = 0;
		forward_right_pwm = 0;
	}
	*/
}

void stop(int time)
{

	//displayMatrix("STOP");

	targetLeft = 0;

	targetRight = 0;

	delay_ms(time);

}



void forwardDistance(int distance, int left_speed, int right_speed, bool coast) {
	int curEnc = getLeftEncCount();
	turning = 0;
	while (getLeftEncCount() - curEnc < distance) {
		left_enc = getLeftEncCount();
		right_enc = getRightEncCount(); 
		//printf("Encoder counts left: %d\r\n",distance - getLeftEncCount() + curEnc); 
		//displayMatrix("FWD");
		targetLeft = left_speed;
		targetRight = right_speed;

	}

	if (!coast) {
		targetLeft = 0;
		targetRight = 0;
	}
	turning = 1;
}



void turnDegrees(int degrees, int direction, int speed) {
	int curAng;
	curAng = angle;
	turning = 1;
	if(direction == 1)
	{
		while (angle-curAng < degrees) {
			//displayMatrix("LEFT");
			//printf("angle: %d\tcurAngle: %d\tangle-curAngle: %d\r\n", angle, curAng, angle-curAng); 
			targetLeft = -speed*direction;
			targetRight = speed*direction;
		}
	}
	else if(direction == -1)
	{
			while (angle-curAng > -degrees) {
			//displayMatrix("RIGT");
			//printf("angle: %d\tcurAngle: %d\tangle-curAngle: %d\r\n", angle, curAng, angle-curAng); 
			targetLeft = -speed*direction;
			targetRight = speed*direction;
		}
	}

	targetLeft = 0;
	targetRight = 0;
	

}

void forwardTurn(int direction, int runSpeed) {
	if (direction == RIGHT) {
					displayMatrix("RIGT");
					forwardDistance(2600, runSpeed, runSpeed, false);
		stop(250);
					turnDegrees(14000, RIGHT, 50);
		stop(250);
					forwardDistance(1600, runSpeed, runSpeed, false);	
	}
	else {
	forwardDistance(2600, runSpeed, runSpeed, false);
		stop(250);
	turnDegrees(14000, direction, 60);
		stop(250);
	forwardDistance(1600, runSpeed, runSpeed, false);
	}
}



///////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////


void systick(void) {

	if (mouseStarted == 1) {
		readGyro();
		readVolMeter();
		readSensor();
		PID();
		left_enc = getLeftEncCount();
		right_enc = getRightEncCount();  
	}
}

///////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////

void button1_interrupt(void) {
	readSensor();
	displayInt(LFSensor);
}


void button2_interrupt(void) {
	readSensor();
			if((DLSensor > hasLeftWall) && (DRSensor > hasRightWall))//has both walls
			{
				displayMatrix("BOTH");
			}        
			else if((DLSensor > hasLeftWall))//only has left wall
			{
				
				displayMatrix("LWAL");
			}
			else if((DRSensor > hasRightWall))//only has right wall
			{
				displayMatrix("RWAL");
			}
			else if((DLSensor < hasLeftWall && DRSensor <hasRightWall))//no wall, use encoder or gyro
			{
				displayMatrix("NONE");
			}
}



int main(void) {
	
	int runSpeed = 50;
	int i;
	int j;
	Systick_Configuration();
	LED_Configuration();
	button_Configuration();
	usart1_Configuration(9600);
	SPI_Configuration();
  TIM4_PWM_Init();
	Encoder_Configration();
	buzzer_Configuration();
	ADC_Config();
	calculateManahattan(xGOAL, yGOAL);
	
	//shortBeep(2000, 8000);

	stop(1000);
	
	heading = NORTH;

	displayMatrix("CALB");
	for (i = 0; i < 1000; i++) {
		readSensor();
		leftMiddleValue += DLSensor;
		rightMiddleValue += DRSensor;
	}
	leftMiddleValue /= 1000;
	rightMiddleValue /= 1000;

	hasLeftWall = leftMiddleValue * 0.55;
	hasRightWall = rightMiddleValue * 0.55;
	

	//forwardDistance(200, runSpeed, runSpeed, true);
	
	stop(100);
	mouseStarted = 1;
	printf("Starting run\r\n");
	
	/////////test///////////
	setV(0,0);
	forwardDistance(leftEncoderDeltaCellInit, runSpeed, runSpeed, false);
	stop(100);
	
	//////////////////
	
while(1) {	
	//while (RFSensor < hasFrontWallLeft || LFSensor < hasFrontWallRigh t) {		
	
	/*displayMatrix("LOOP");
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	switch(heading) {
			case NORTH:
				printf("Current Heading: North");
					break;
			case SOUTH:
				printf("Current Heading: South");
					break;
			case EAST:
				printf("Current Heading: East");
					break;
			case WEST:
				printf("Current Heading: West");
					break;
			case INVALID:
			default:
				break;
			}*/	
	/*
	printf("X: %d Y: %d\r\n",x, y); 
	printf("hasRightWall: %d RightSensorValue: %d\r\n",hasRightWall, DRSensor);
	printf("hasLeftWall: %d LeftSensorValue: %d\r\n",hasLeftWall, DLSensor);
	printf("hasFrontWall: %d LeftSensorValue: %d\r\n",hasFrontWallLeft, LFSensor);
	printf("hasFrontWall: %d RightSensorValue: %d\r\n",hasFrontWallRight, RFSensor);
	
	*/
	
		//Set the direction to turn
		//printf("Looking ahead: ");
			displayMatrix("HEAD");
		if((DLSensor > hasLeftWall)) {
			//printf("Left Wall\t");
			leftWall = true;
		} else {
			//printf("NO Left Wall\t");
			leftWall = false;
		}
				
		if((DRSensor > hasRightWall)) {
			//printf("Right Wall\t");
			rightWall = true;
		} else {
			//printf("NO Right Wall\t");
			rightWall = false;
		}
				
		if((LFSensor > hasFrontWallLeft || RFSensor > hasFrontWallRight)) {
			//printf("Front Wall\r\n");
			frontWall = true;
		} else {
			//printf("NO Front Wall\r\n");
			frontWall = false;
		}
		displayMatrix("NEXT");
		if (leftWall && rightWall && !frontWall) {
			forwardDistance(leftEncoderDeltaCell, runSpeed, runSpeed, true);
		}
		else if (leftWall && !rightWall) {
			forwardTurn(RIGHT, 50);
		}
		else if (rightWall && !leftWall) {
			forwardTurn(LEFT, 50);
		}
		else if (leftWall && rightWall && frontWall) {
			turnDegrees(30000, LEFT, 50);
		}
		else if (frontWall) {
			forwardTurn(LEFT, 50);
		}
		//stop(500);
		/*i = nextMovement();
		//printf("Next Movement is %d\r\n*********************************\r\n", i);
		switch (i) {
			case MoveForward: 
				switch(heading) {
					case NORTH:
					y++;
					break;
				case SOUTH:
					y--;
					break;
				case EAST:
					x++;
					break;
				case WEST:
					x--;
					break;
				case INVALID:
				default:
					break;
			}
			displayMatrix("FWD");
			forwardDistance(leftEncoderDeltaCell, runSpeed, runSpeed, true);
			break;
								
			case TurnClockwise:
						
						switch(heading) {
							case NORTH:
								y++;
								break;
						case SOUTH:
								y--;
								break;
						case EAST:
								x++;
								break;
						case WEST:
								x--;
								break;
						case INVALID:
						default:
							break;
						}	
					turning = 1;
					forwardTurn(RIGHT, runSpeed);
						stop(250);
					break;
					
			case TurnCounterClockwise:
						switch(heading) {
							case NORTH:
								y++;
								break;
						case SOUTH:
								y--;
								break;
						case EAST:
								x++;
								break;
						case WEST:
								x--;
								break;
						case INVALID:
						default:
							break;
						}	
						
						turning = 1;
						forwardTurn(LEFT, runSpeed);
						
						break;

			case TurnAround:
					displayMatrix("BACK");
					turnDegrees(34000, LEFT, 50);
					
						break; 
				}
		stop(500);
		displayMatrix("REAC");*/
				
		/*		stop(100);
				//displayMatrix("PRNT");
				//Print out Manhattan Distances
				for (i = 15; i >= 0; i--) {
					for (j = 0; j < 16; j++) {
						
						printf("%d ", c_array[j][i].m_distance);
					}
					printf("\r\n");
				}
				//Print out walls
				for (i = 15; i >= 0; i--) {
					for (j = 0; j < 16; j++) {
						printf("*");
						if (getH(j, i)) {
							printf("-");
						}
						else {
							printf(" ");
						}
					}
					printf("\r\n");
					printf(" ");
					for (j = 0; j < 16; j++) {
						printf(" ");
						if (getV(j, i)) {
							printf("|");
						}
					}
					printf("\r\n");
				}
				printf("\r\n\r\n");
			readSensor();
			if((DLSensor > hasLeftWall) && (DRSensor > hasRightWall))//has both walls
			{
				displayMatrix("BOTH");
			}        
			else if((DLSensor > hasLeftWall))//only has left wall
			{
				
				displayMatrix("LWAL");
			}
			else if((DRSensor > hasRightWall))//only has right wall
			{
				displayMatrix("RWAL");
			}
			else if((DLSensor < hasLeftWall && DRSensor <hasRightWall))//no wall, use encoder or gyro
			{
				displayMatrix("NONE");
			}		*/
			//stop(1000);
				
				
				//printf("=================================\n=================================\r\n\r\n");
			
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/*
			//readSensor();
			if((DLSensor > hasLeftWall) && (DRSensor > hasRightWall))//has both walls
			{
				displayMatrix("BOTH");
			}        
			else if((DLSensor > hasLeftWall))//only has left wall
			{
				
				displayMatrix("LWAL");
			}
			else if((DRSensor > hasRightWall))//only has right wall
			{
				displayMatrix("RWAL");
			}
			else if((DLSensor < hasLeftWall && DRSensor <hasRightWall))//no wall, use encoder or gyro
			{
				displayMatrix("NONE");
			}		
			printf("LeftFront: %d\tRightFront: %d\r\n",LFSensor, RFSensor);
			//delay_ms(1000);
			//while (LFSensor > hasFrontWallLeft && RFSensor > hasFrontWallRight) {
			printf("About to go forward\r\n");
			forwardDistance(leftEncoderDeltaCell, runSpeed, runSpeed, true);
			printf("finished going forward\r\n");
			//}
			//stop(10000);
			//delay_ms(500);
			
			
		}

		
		printf("found wall. out of while loop\r\n");
		displayMatrix("DONE");
		targetLeft = 0;
		targetRight = 0;
		
		stop(1000);
		turnDegrees(-14000, -1, 50);
		stop(1000);
		turnDegrees(-14000, -1, 50);
		stop(1000);
	}
		
		while(1) {
			delay_ms(1);
		}
		
		*/
		/*buttonPressed = 1;
		stop(1000);
		while (1) {
			if (buttonPressed == 0){
				delay_ms(500);
				break;
			}
		}*/
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	}
while(1) {
			readSensor();
	//shortBeep(100, 5000);
			if((DLSensor > hasLeftWall) && (DRSensor > hasRightWall))//has both walls
			{
				displayMatrix("BOTH");
			}        
			else if((DLSensor > hasLeftWall))//only has left wall
			{
				
				displayMatrix("LWAL");
			}
			else if((DRSensor > hasRightWall))//only has right wall
			{
				displayMatrix("RWAL");
			}
			else if((DLSensor < hasLeftWall && DRSensor <hasRightWall))//no wall, use encoder or gyro
			{
				displayMatrix("NONE");
			}		
			delay_ms(500);
		}
}



