/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De and Turner Topping <avik@ghostrobotics.io> <turner@ghostrobotics.io>
 * 
 * 
 * Zhiyi (Allen) Ren
 * all four legs swinging, same phase, for Nelder-Mead data collection
 */
#include <stdio.h>
#include <SDK.h>
#include <math.h>
#include <Motor.h>
#include <ReorientableBehavior.h>

#include <Interpolator.h>
#include <func.h>

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
const int   t_side   = 200;
const int   t_bottom = 300;
const int   period = 2*(t_bottom+t_side);

int offset = period/2;  // walking

// State machine representation of behavior
enum FHMode
{
	FH_SIT = 0,
	FH_STAND,
	FH_START,
};

class TriTrajWalk : public ReorientableBehavior
{
public:
	FHMode mode = FH_SIT; // Current state within state-machine

	bool start = true;
	int startTimeResidual;
	int t;
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
		C->mode = RobotCommand_Mode_JOINT;

		if (mode == FH_STAND)
		{
			ioctl(LOGGER_FILENO, 0); // start logging
			start = true;
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
			
			t = (S->millis - startTimeResidual) % period;

			for(int i=0; i<8; i++)
			{
				if( i==1 || i==6 || i==4 || i==3 )  // in
				{
				  pwm[i] = kph*(qIn[t]-joint[i].getPosition())+kd*(0-joint[i].getVelocity());
				}
				else if( i==0 || i==7 || i==5 || i==2 )  // out
				{
					pwm[i] = kph*(qOut[t]-joint[i].getPosition())+kd*(0-joint[i].getVelocity());
				}
			}

			for(int i=0; i<8; i++)
			{
				joint[i].setOpenLoop(constrain(pwm[i],-1,1));
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
	// myData_buf[0] = joint[1].getOpenLoop();
	// myData_buf[1] = joint[0].getOpenLoop();
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

	TriTrajWalk triTrajWalk;
	behaviors.push_back(&triTrajWalk);

	return begin();
}
