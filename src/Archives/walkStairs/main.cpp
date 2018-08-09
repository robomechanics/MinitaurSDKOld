/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De and Turner Topping <avik@ghostrobotics.io> <turner@ghostrobotics.io>
 * 
 * 
 * Zhiyi (Allen) Ren
 * 
 */
#include <stdio.h>
#include <SDK.h>
#include <math.h>
#include <Motor.h>
#include <ReorientableBehavior.h>

#include <Interpolator.h>
#include <func.h>
#include <obsValue.h>

#include <string>
#include <iostream>

#if defined(ROBOT_MINITAUR)
// Subject to change for individual robots
// const float motZeros[8] = {2.82, 3.435, 3.54, 3.076, 1.03, 3.08, 6.190, 1.493};
// const float motZeros[8] = {2.570, 3.167, 3.777, 3.853, 2.183, 1.556, .675, 2.679}; // RML Ellie
const float motZeros[8] = {2.570, 3.190, 3.777, 3.853, 2.183, 1.556, .675, 2.679}; // RML Ellie corrected

// const float motZeros[8] = {0.631, 4.076, 1.852, 3.414, 1.817, 5.500, 1.078, 6.252}; //RML Odie
#endif

char myData[32];
float *myData_buf = (float*)myData;

const float apExt = 0.12;
const float tdExt = 0.18;
const float angle = -15;
const int   t_side   = 100;
const int   t_bottom = 300;
const int   numPointsCycle = 2*(t_bottom+t_side);
const int   period         = numPointsCycle;

float x[numPointsCycle];
float y[numPointsCycle];

float height;

int offset = period/2;  // walking

// State machine representation of behavior
enum FHMode
{
	FH_SIT = 0,
	FH_STAND,
	// FH_START,
	FH_FU1
};

class WalkStairs : public ReorientableBehavior
{
public:
	FHMode mode = FH_SIT; // Current state within state-machine

	bool start = true;
	int startTimeResidual;
	int t;
	int t_left = 0;
	int t_right = 0;
	float step = 0.001;

	// float x[numPointsCycle];
	// float y[numPointsCycle];
	float qIn[numPointsCycle];
	float qOut[numPointsCycle];
	float integral[8] = {0,0,0,0,0,0,0,0};
	float pwm[8];

	float kpl = 0.3;
	float kpm = 0.6;
	float kph = 0.9;
	float kd = 0.01;
	float ki = 0.1;

	int fu1_state = 1;
	// bool recov = false;
	// int turn = 1;  // start with left
	// bool leftTurn = 0;
	// int wait_time = 0;
	// int pre_time = 0;

	// float *q[8];

	// int r[2] = {0,3};

	// use joystick to start walking or stop
	void signal(uint32_t sig)
	{
		if (sig == 3)
		{
			mode = FH_FU1;
		}
		else
		{
			mode = FH_STAND;
		}
	}

	void begin()
	{
		mode = FH_STAND;			// Start behavior in STAND mode

		// getTrajInterp(qIn, qOut, apExt, tdExt, angle, t_bottom, t_side);
		// float *qIn

	}

	void update()
	{
		ioctl(LOGGER_FILENO, 0); // start logging
		C->mode = RobotCommand_Mode_LIMB;

		if (mode == FH_STAND)
		{
			fu1_state = 1;
			for(int i=0; i<4; i++)
			{
				float ang = (isFront(i)) ? -S->imu.euler.y - 0 : -S->imu.euler.y - 0;
				limb[i].setGain(ANGLE, 0.8, 0.02);
				limb[i].setPosition(ANGLE, ang);

				float ext = (isFront(i)) ? 0.28 : 0.28;
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[i].setGain(EXTENSION, 120, 4);
				limb[i].setPosition(EXTENSION, ext);
			}
		}

		else if (mode == FH_FU1)
		{
			ioctl(LOGGER_FILENO, 1); // start logging

			for(int i=0; i<3; i++)  // except for right left
			{
				if(!isFront(i)) continue;
				limb[i].setGain(ANGLE, 0.8, 0.02);
				limb[i].setPosition(ANGLE, -S->imu.euler.y - 0);

				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[i].setGain(EXTENSION, 180, 4);
				limb[i].setPosition(EXTENSION, 0.27);
			}	

			if(fu1_state == 1)
			{
				limb[1].setGain(ANGLE, 0.8, 0.02);
				limb[1].setPosition(ANGLE, 0);

				P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[1].setGain(EXTENSION, 180, 5);
				limb[1].setPosition(EXTENSION, 0.11);

				limb[3].setGain(ANGLE, 0.8, 0.02);
				limb[3].setPosition(ANGLE, -S->imu.euler.y - 0);

				P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[3].setGain(EXTENSION, 120, 4);
				limb[3].setPosition(EXTENSION, 0.27);

				if(limb[1].getPosition(EXTENSION) < 0.12) fu1_state = 2;
			}
			else if(fu1_state == 2)
			{
				limb[1].setGain(ANGLE, 1.2, 0.03);
				limb[1].setPosition(ANGLE, -S->imu.euler.y + 0.7);

				P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[1].setGain(EXTENSION, 120, 4);
				limb[1].setPosition(EXTENSION, 0.11);

				limb[3].setGain(ANGLE, 0.8, 0.02);
				limb[3].setPosition(ANGLE, -S->imu.euler.y - 0);

				P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[3].setGain(EXTENSION, 180, 3);
				limb[3].setPosition(EXTENSION, 0.29);

				if(limb[1].getPosition(ANGLE) > -S->imu.euler.y + 0.6) fu1_state = 3;				
			}
			else if(fu1_state == 3)
			{
				limb[1].setGain(ANGLE, 0.8, 0.02);
				limb[1].setPosition(ANGLE, -S->imu.euler.y + 0.6);

				P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[1].setGain(EXTENSION, 120, 4);
				limb[1].setPosition(EXTENSION, 0.18);

				limb[3].setGain(ANGLE, 0.8, 0.02);
				limb[3].setPosition(ANGLE, -S->imu.euler.y - 0);

				P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[3].setGain(EXTENSION, 180, 5);
				limb[3].setPosition(EXTENSION, 0.11);

				if(limb[3].getPosition(EXTENSION) < 0.12) fu1_state = 4;				
			}
			else if(fu1_state == 4)
			{
				limb[1].setGain(ANGLE, 0.8, 0.02);
				limb[1].setPosition(ANGLE, -S->imu.euler.y + 0.6);

				P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[1].setGain(EXTENSION, 120, 4);
				limb[1].setPosition(EXTENSION, 0.18);

				limb[3].setGain(ANGLE, 1.2, 0.03);
				limb[3].setPosition(ANGLE, -S->imu.euler.y + 0.7);

				P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[3].setGain(EXTENSION, 120, 5);
				limb[3].setPosition(EXTENSION, 0.11);

				if(limb[3].getPosition(ANGLE) > -S->imu.euler.y + 0.6) fu1_state = 5;						
			}	
			else if(fu1_state == 5)
			{
				limb[1].setGain(ANGLE, 0.8, 0.02);
				limb[1].setPosition(ANGLE, -S->imu.euler.y + 0.6);

				P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[1].setGain(EXTENSION, 120, 4);
				limb[1].setPosition(EXTENSION, 0.18);

				limb[3].setGain(ANGLE, 0.8, 0.02);
				limb[3].setPosition(ANGLE, -S->imu.euler.y + 0.6);

				P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[3].setGain(EXTENSION, 120, 4);
				limb[3].setPosition(EXTENSION, 0.18);

				// if(limb[3].getPosition(ANGLE) > -S->imu.euler.y + 0.45) fu1_state = 5;						
			}				
		}
	}

	bool running()
	{
		return !(mode == FH_SIT);
	}
	void end()
	{
		mode = FH_SIT;
	}
};

void debug()
{
	// myData_buf[0] = joint[0].getOpenLoop();
	// myData_buf[1] = joint[1].getOpenLoop();
	// myData_buf[2] = joint[2].getOpenLoop();
	// myData_buf[3] = joint[3].getOpenLoop();
	// myData_buf[4] = joint[4].getOpenLoop();
	// myData_buf[5] = joint[5].getOpenLoop();
	// myData_buf[6] = joint[6].getOpenLoop();
	// myData_buf[7] = joint[7].getOpenLoop();

	// write(LOGGER_FILENO, myData, 32);
}

int main(int argc, char *argv[])
{
	// MINITAUR
	init(RobotParams_Type_MINITAUR, argc, argv);
	for (int i = 0; i < P->joints_count; ++i)
		P->joints[i].zero = motZeros[i];

	// for SD card logging
  setDebugRate(100);

	WalkStairs walkStairs;
	behaviors.push_back(&walkStairs);

	return begin();
}
