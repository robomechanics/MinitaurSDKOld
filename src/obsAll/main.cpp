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
#include <observer.h>

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

float pos_act[8] = {0,0,0,0,0,0,0,0};
float vel_act[8] = {0,0,0,0,0,0,0,0};
float pos_obs_int[8] = {0,0,0,0,0,0,0,0};
float vel_obs_int[8] = {0,0,0,0,0,0,0,0};
float pos_obs[8] = {0,0,0,0,0,0,0,0};
float vel_obs[8] = {0,0,0,0,0,0,0,0};
float batt_act = 0;

float res[2] = {0,0,0,0,0,0,0,0};

// State machine representation of behavior
enum FHMode
{
	FH_SIT = 0,
	FH_STAND,
	FH_START
};

class ObsAll : public ReorientableBehavior
{
public:
	FHMode mode = FH_SIT; // Current state within state-machine

	bool start = true;
	int startTimeResidual;

	int tLeft, tRight;
	float qIn[numPointsCycle];
	float qOut[numPointsCycle];
	float pwm[8];
	float integral[8];

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
		ioctl(LOGGER_FILENO, 0); // no logging yet
		mode = FH_STAND;			// Start behavior in STAND mode

		getTrajInterp(qIn, qOut, apExt, tdExt, angle, t_bottom, t_side);
	}

	void update()
	{
		// ioctl(LOGGER_FILENO, 1); // start logging
		C->mode = RobotCommand_Mode_JOINT;

		if (mode == FH_STAND)
		{
			ioctl(LOGGER_FILENO, 0); // do not log
			// use PID to achieve accurate starting positions
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

			if(start)
			{
				ioctl(LOGGER_FILENO, 0); // do jot log
				// copy initial conditions
				pos_obs[0] = toT1(joint[1].getPosition());
				pos_obs[1] = toT2(joint[0].getPosition());
				pos_obs[2] = toT1(joint[3].getPosition());
				pos_obs[3] = toT2(joint[2].getPosition());
				pos_obs[4] = toT1(joint[4].getPosition());
				pos_obs[5] = toT2(joint[5].getPosition());
				pos_obs[6] = toT1(joint[6].getPosition());
				pos_obs[7] = toT2(joint[7].getPosition());

				vel_obs[0] = joint[1].getVelocity();
				vel_obs[1] = -joint[0].getVelocity();	
				vel_obs[2] = joint[3].getVelocity();
				vel_obs[3] = -joint[2].getVelocity();
				vel_obs[4] = joint[4].getVelocity();
				vel_obs[5] = -joint[5].getVelocity();	
				vel_obs[6] = joint[6].getVelocity();
				vel_obs[7] = -joint[7].getVelocity();	

				startTimeResidual = S->millis % period;
				start = false;
			}
			else
			{
				ioctl(LOGGER_FILENO, 1); // start logging
			}

			tLeft  = (S->millis - startTimeResidual) % period;  // correct the starting time
			tRight = (S->millis - startTimeResidual + period/2) % period;  // correct the starting time

			// actual
			if((t % 10) == 0 && !start)
			{
				batt_act = S->batt.voltage;

				// copy over
				for(int i=0; i<8; i++)
				{
					pos_obs[i] = pos_obs_int[i];
					vel_obs[i] = vel_obs_int[i];
				}
				// get new value
				pos_obs_int[0] = toT1(joint[1].getPosition());
				pos_obs_int[1] = toT2(joint[0].getPosition());
				pos_obs_int[2] = toT1(joint[3].getPosition());
				pos_obs_int[3] = toT2(joint[2].getPosition());
				pos_obs_int[4] = toT1(joint[4].getPosition());
				pos_obs_int[5] = toT2(joint[5].getPosition());
				pos_obs_int[6] = toT1(joint[6].getPosition());
				pos_obs_int[7] = toT2(joint[7].getPosition());

				vel_obs_int[0] = joint[1].getVelocity();
				vel_obs_int[1] = -joint[0].getVelocity();	
				vel_obs_int[2] = joint[3].getVelocity();
				vel_obs_int[3] = -joint[2].getVelocity();
				vel_obs_int[4] = joint[4].getVelocity();
				vel_obs_int[5] = -joint[5].getVelocity();	
				vel_obs_int[6] = joint[6].getVelocity();
				vel_obs_int[7] = -joint[7].getVelocity();	

				// calculate residual
				// res[0] = pos_obs[0] - pos_obs_int[0];
				// res[1] = pos_obs[1] - pos_obs_int[1];
 			}

			// run robot
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

			// run all four observers
			obs(pos_obs_int, vel_obs_int, batt_act, joint[1].getOpenloop(), joint[0].getOpenLoop(), 0);
			obs(pos_obs_int, vel_obs_int, batt_act, joint[3].getOpenLoop(), joint[2].getOpenLoop(), 1);
			obs(pos_obs_int, vel_obs_int, batt_act, joint[4].getOpenloop(), joint[5].getOpenLoop(), 2);
			obs(pos_obs_int, vel_obs_int, batt_act, joint[6].getOpenloop(), joint[7].getOpenLoop(), 3);
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

	myData_buf[0] = toQ1(pos_obs[0]);
	myData_buf[1] = toQ2(pos_obs[1]);
	myData_buf[2] = vel_obs[0];
	myData_buf[3] = -vel_obs[1];
	myData_buf[4] = toQ1(pos_obs[4]);
	myData_buf[5] = toQ2(pos_obs[5]);
	myData_buf[6] = vel_obs[4];
	myData_buf[7] = -vel_obs[5];
	// myData_buf[4] = res[0];
	// myData_buf[5] = res[1];

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

	ObsAll obsAll;
	behaviors.push_back(&obsAll);

	return begin();
}
