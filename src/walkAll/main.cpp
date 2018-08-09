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
const float motZeros[8] = {2.570, 2.036, 3.777, 3.853, 2.183, 1.556, .675, 2.679}; // RML Ellie
// const float motZeros[8] = {0.631, 4.076, 1.852, 3.414, 1.817, 5.500, 1.078, 6.252}; //RML Odie
#endif

char myData[32];
float *myData_buf = (float*)myData;

const float apExt = 0.12;
const float tdExt = 0.18;
const float angle = -15;
const int   t_side   = 100;
const int   t_bottom = 300;
const int   period = 2*(t_bottom+t_side);

int offset = period/2;  // walking

// State machine representation of behavior
enum FHMode
{
	FH_SIT = 0,
	FH_STAND,
	FH_START,
	FH_STOP
};

float res_pos[8] = {0,0,0,0,0,0,0,0};
float res_pos_avg[4] = {0,0,0,0};

class WalkAll : public ReorientableBehavior
{
public:
	FHMode mode = FH_SIT; // Current state within state-machine

	bool start = true;
	bool wait = false;
	int startTimeResidual;
	int startTime = 0;
	int waitTime = 0;
	int t;
	int t_left = 0;
	int t_right = 0;
	float step = 0.001;

	float qIn[period];
	float qOut[period];
	float integral[8] = {0,0,0,0,0,0,0,0};
	float pwm[8];

	float kpl = 0.3;
	float kpm = 0.6;
	float kph = 0.9;
	float kd = 0.01;
	float ki = 0.1;

	// use joystick to start walking or stop
	void signal(uint32_t sig)
	{
		if (sig == 3)
		{
			mode = FH_START;
		}
		else
		{
			mode = FH_STAND;
		}
	}

	void begin()
	{
		mode = FH_STAND;			// Start behavior in STAND mode
		getTrajInterp(qIn, qOut, apExt, tdExt, angle, t_bottom, t_side);
	}

	void update()
	{
		// ioctl(LOGGER_FILENO, 0); // start logging
		C->mode = RobotCommand_Mode_JOINT;

		if (mode == FH_STAND || mode == FH_STOP)
		{
			ioctl(LOGGER_FILENO, 0); // start logging
			// reset walking parameters
			start = true;
			wait = false;
			startTime = 0;
			waitTime = 0;

			for(int i=0; i<8; i++)
			{
				if( i==1 || i==6 || i==4 || i==3 )  // in
				{
					integral[i] += step*(qIn[0]-joint[i].getPosition());
				  pwm[i] = kpm*(qIn[0]-joint[i].getPosition())+kd*(0-joint[i].getVelocity()+ki*integral[i]);
				}
				else if( i==0 || i==7 || i==5 || i==2 )  // out
				{
				  integral[i] += step*(qOut[0]-joint[i].getPosition());
					pwm[i] = kpm*(qOut[0]-joint[i].getPosition())+kd*(0-joint[i].getVelocity()+ki*integral[i]);
				}
			}

			for(int i=0; i<8; i++)
			{
				joint[i].setOpenLoop(constrain(pwm[i],-1,1));
			}
		}

		else if (mode == FH_START)
		{
			ioctl(LOGGER_FILENO, 1); // start logging

			if(start)
			{
				startTimeResidual = S->millis % period;
				start = false;
			}
			
			t_left = (S->millis - startTimeResidual) % period;
			t_right = (S->millis - startTimeResidual + offset) % period;

			for(int i=0; i<8; i++)
			{
				if( i==1 || i==6 )  // left in
				{
					pwm[i] = kph*(qIn[t_left]-joint[i].getPosition())+kd*(0-joint[i].getVelocity());
				}
				else if( i==0 || i==7 )  // left out
				{
					pwm[i] = kph*(qOut[t_left]-joint[i].getPosition())+kd*(0-joint[i].getVelocity());
				}
				else if( i==4 || i==3 )  // right in
				{
					pwm[i] = kph*(qIn[t_right]-joint[i].getPosition())+kd*(0-joint[i].getVelocity());
				}
				else if( i==5 || i==2 ) // right out
				{
					pwm[i] = kph*(qOut[t_right]-joint[i].getPosition())+kd*(0-joint[i].getVelocity());
				}
			}

			for(int i=0; i<8; i++)
			{
				joint[i].setOpenLoop(constrain(pwm[i],-1,1));
			}

			if((++startTime) < 500)
			{
				mode = FH_START;
				return;  // ignore first 500ms for detection
			}

			res_pos[6] = abs(joint[6].getPosition() - q1_obs[t_left - 350]);
			res_pos[7] = abs(joint[7].getPosition() - q2_obs[t_left - 350]);
			res_pos_avg[3] = (res_pos[6]+res_pos[7])*0.5;
			res_pos[3] = abs(joint[3].getPosition() - q1_obs[t_right - 350]);
			res_pos[2] = abs(joint[2].getPosition() - q2_obs[t_right - 350]);
			res_pos_avg[1] = (res_pos[3]+res_pos[2])*0.5;

			if(wait)
			{
				if((++waitTime) > 30) mode = FH_STOP;
				return;
			}

			// check hit
			if(t_left >= 400 && t_left <= 465 && res_pos_avg[3] > 0.05)  // no differentiation between stair and obstacle for now
			{
				wait = true;
			}
			if(t_right >= 400 && t_right <= 465 && res_pos_avg[1] > 0.05)  // no differentiation between stair and obstacle for now
			{
				wait = true;
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
	myData_buf[0] = res_pos_avg[0];
	myData_buf[1] = res_pos_avg[1];
	myData_buf[2] = res_pos_avg[2];
	myData_buf[3] = res_pos_avg[3];

	write(LOGGER_FILENO, myData, 32);
}

int main(int argc, char *argv[])
{
	// MINITAUR
	init(RobotParams_Type_MINITAUR, argc, argv);
	for (int i = 0; i < P->joints_count; ++i)
		P->joints[i].zero = motZeros[i];

	// for SD card logging
  setDebugRate(100);

	WalkAll walkAll;
	behaviors.push_back(&walkAll);

	return begin();
}
