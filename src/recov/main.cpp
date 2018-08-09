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
const float motZeros[8] = {2.570, 2.036, 3.777, 3.853, 2.183, 1.556, .675, 2.679}; // RML Ellie corrected

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

float x[period];
float y[period];

int offset = period/2;  // walking

// State machine representation of behavior
enum FHMode
{
	FH_SIT = 0,
	FH_STAND,
	FH_START,
};

class TriTrajWalkRecov : public ReorientableBehavior
{
public:
	FHMode mode = FH_SIT; // Current state within state-machine

	bool start = true;
	int startTimeResidual;
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

	float res[8];

	int recov_time = 0;
	bool recov = false;
	// int turn = 1;  // start with left
	// bool leftTurn = 0;
	// int wait_time = 0;
	// int pre_time = 0;

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
		ioctl(LOGGER_FILENO, 0); // start logging
		C->mode = RobotCommand_Mode_JOINT;

		if (mode == FH_STAND)
		{
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

			// if(turn == 1)  // left move
			// {
			// 	t_right = 0;  // stance
			// 	if((++t_left) == (period-1)) turn = 2;
			// }
			// else if(turn  == 2)  // right move
			// {
			// 	t_left = 0;
			// 	if((++t_right) == (period-1)) turn = 1;
			// }
			// else{  // wait
			// 	t_left = 0;
			// 	t_right = 0;
			// 	if((++wait) == 200)
			// 	{
			// 		leftTurn ? turn = 2 : turn = 1;
			// 		leftTurn = !leftTurn;
			// 		wait = 0;
			// 	}
			// }
			
			t_left = (S->millis - startTimeResidual) % period;
			t_right = (S->millis - startTimeResidual + offset) % period;

			// check collision
			if(t_left >= 350 && t_left <= 450 && recov == false)
			{
				res[0] = abs(joint[0].getPosition() - q2_obs[t_left - 350]);
				res[1] = abs(joint[1].getPosition() - q1_obs[t_left - 350]);
				if(((res[0]+res[1])*0.5) > 0.05) recov = true;
			}

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

			if(recov)
			{
				recov_time++;
				if(recov_time <= 45)
				{
					float out[2];
					ik(out, 0.03, -0.22);  // backwards a bit

					pwm[0] = kph*(out[1]-joint[0].getPosition())+kd*(0-joint[0].getVelocity());
					pwm[1] = kph*(out[0]-joint[1].getPosition())+kd*(0-joint[1].getVelocity());			
				}
				else if(recov_time <= 90 && recov_time > 45)
				{
					float out[2];
					ik(out, 0, -0.11);

					pwm[0] = kph*(out[1]-joint[0].getPosition())+kd*(0-joint[0].getVelocity());
					pwm[1] = kph*(out[0]-joint[1].getPosition())+kd*(0-joint[1].getVelocity());			
				}
				else if(recov_time <= 150 && recov_time > 90)
				{
					float out[2];
					ik(out, -0.05, -0.18);

					// soft landing
					pwm[0] = kpl*(out[1]-joint[0].getPosition())+kd*(0-joint[0].getVelocity());
					pwm[1] = kpl*(out[0]-joint[1].getPosition())+kd*(0-joint[1].getVelocity());				
				}
				else
				{
					recov = false;
					recov_time = 0;
				}

				// // 2 and 3 extends
				// float out[2];
				// fk(out, qIn[t_right], qOut[t_right]);
				// out[1] -= 0.03;  // y extends
				// ik(out, out[0], out[1]);
				// pwm[2] = kph*(out[1]-joint[2].getPosition())+kd*(0-joint[2].getVelocity());
				// pwm[3] = kph*(out[0]-joint[3].getPosition())+kd*(0-joint[3].getVelocity());

				// // left side soften
				// pwm[6] = kpl*(qIn[t_left]-joint[6].getPosition())+kd*(0-joint[6].getVelocity());
				// pwm[7] = kpl*(qOut[t_left]-joint[7].getPosition())+kd*(0-joint[7].getVelocity());
				// pwm[4] = kpl*(qIn[t_right]-joint[4].getPosition())+kd*(0-joint[6].getVelocity());
				// pwm[5] = kpl*(qOut[t_right]-joint[5].getPosition())+kd*(0-joint[5].getVelocity());

			// }
			// else
			// {
			// 	for(int i=0; i<8; i++)
			// 	{
			// 		if( i==1 || i==6 )  // left in
			// 		{
			// 			pwm[i] = kph*(qIn[t_left]-joint[i].getPosition())+kd*(0-joint[i].getVelocity());
			// 		}
			// 		else if( i==0 || i==7 )  // left out
			// 		{
			// 			pwm[i] = kph*(qOut[t_left]-joint[i].getPosition())+kd*(0-joint[i].getVelocity());
			// 		}
			// 		else if( i==4 || i==3 )  // right in
			// 		{
			// 			pwm[i] = kph*(qIn[t_right]-joint[i].getPosition())+kd*(0-joint[i].getVelocity());
			// 		}
			// 		else if( i==5 || i==2 ) // right out
			// 		{
			// 			pwm[i] = kph*(qOut[t_right]-joint[i].getPosition())+kd*(0-joint[i].getVelocity());
			// 		}
			// 	}
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
	myData_buf[0] = joint[0].getOpenLoop();
	myData_buf[1] = joint[1].getOpenLoop();
	myData_buf[2] = joint[2].getOpenLoop();
	myData_buf[3] = joint[3].getOpenLoop();
	myData_buf[4] = joint[4].getOpenLoop();
	myData_buf[5] = joint[5].getOpenLoop();
	myData_buf[6] = joint[6].getOpenLoop();
	myData_buf[7] = joint[7].getOpenLoop();

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

	TriTrajWalkRecov triTrajWalkRecov;
	behaviors.push_back(&triTrajWalkRecov);

	return begin();
}
