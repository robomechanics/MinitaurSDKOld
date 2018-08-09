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
#endif

char myData[32];
float *myData_buf = (float*)myData;

const float apExt = 0.12;
const float tdExt = 0.18;
const float angle = -15;
const int   t_side   = 100;
const int   t_bottom = 300;
const int   period   = 2*(t_bottom+t_side);

float height;

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

	bool hit[4] = {false, false, false, false};
	bool miss[4] = {false, false, false, false};
	float res[8];

	int recov_time[4] = {0,0,0,0};
	// bool recov = false;
	// int turn = 1;  // start with left
	// bool leftTurn = 0;
	// int wait_time = 0;
	// int pre_time = 0;

	// float *q[8];

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
		// float *qIn
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
			
			t_left = (S->millis - startTimeResidual) % period;
			t_right = (S->millis - startTimeResidual + offset) % period;
			// float times[8] = {t_left, t_left, t_right, t_right, t_right, t_right, t_left, t_left};

			// calculate height

			// check hit
			if(t_left >= 400 && t_left <= 450)  // second half of air phase
			{
				if(!hit[0] && !miss[0]) // check limb 0
				{
					res[0] = abs(joint[0].getPosition() - q2_obs[t_left - 340]);
					res[1] = abs(joint[1].getPosition() - q1_obs[t_left - 340]);
					if(((res[0]+res[1])/2.0) > 0.05) hit[0] = true;
				}
				if(!hit[3] && !miss[3]) // check limb 3
				{
					res[7] = abs(joint[7].getPosition() - q2_obs[t_left - 340]);
					res[6] = abs(joint[6].getPosition() - q1_obs[t_left - 340]);
					if(((res[7]+res[6])/2.0) > 0.05) hit[3] = true;
				}
			}

			if(t_right >= 400 && t_right <= 450)
			{
				if(!hit[1] && !miss[1]) // check limb 0
				{
					res[2] = abs(joint[2].getPosition() - q2_obs[t_right - 340]);
					res[3] = abs(joint[3].getPosition() - q1_obs[t_right - 340]);
					if(((res[2]+res[3])/2.0) > 0.05) hit[1] = true;
				}
				if(!hit[2] && !miss[2]) // check limb 3
				{
					res[5] = abs(joint[5].getPosition() - q2_obs[t_right - 340]);
					res[4] = abs(joint[4].getPosition() - q1_obs[t_right - 340]);
					if(((res[5]+res[4])/2.0) > 0.05) hit[2] = true;
				}
			}

			// // check miss
			// if(t_left >= 480 && t_left <= 550)  // TD
			// {
			// 	if(!hit[0] && !miss[0]) // check limb 0
			// 	{
			// 		res[0] = abs(joint[0].getPosition() - q2_obs[t_left - 340]);
			// 		res[1] = abs(joint[1].getPosition() - q1_obs[t_left - 340]);
			// 		if(((res[0]+res[1])/2.0) < 0.05) miss[0] = true;
			// 	}
			// 	if(!hit[3] && !miss[3]) // check limb 3
			// 	{
			// 		res[7] = abs(joint[7].getPosition() - q2_obs[t_left - 340]);
			// 		res[6] = abs(joint[6].getPosition() - q1_obs[t_left - 340]);
			// 		if(((res[7]+res[6])/2.0) < 0.05) miss[3] = true;
			// 	}
			// }

			// if(t_right >= 480 && t_right <= 550)
			// {
			// 	if(!hit[1] && !miss[1]) // check limb 0
			// 	{
			// 		res[2] = abs(joint[2].getPosition() - q2_obs[t_right - 340]);
			// 		res[3] = abs(joint[3].getPosition() - q1_obs[t_right - 340]);
			// 		if(((res[2]+res[3])/2.0) < 0.05) miss[1] = true;
			// 	}
			// 	if(!hit[2] && !miss[2]) // check limb 3
			// 	{
			// 		res[5] = abs(joint[5].getPosition() - q2_obs[t_right - 340]);
			// 		res[4] = abs(joint[4].getPosition() - q1_obs[t_right - 340]);
			// 		if(((res[5]+res[4])/2.0) < 0.05) miss[2] = true;
			// 	}
			// }

			for(int i=0; i<4; i++)
			{
				if(hit[i])
				{
					recov_time[i]++;
					float out[2];
					float kp_chosen;
					if(recov_time[i] <= 45)
					{
						ik(out, 0.03, -0.22);  // backwards a bit
						kp_chosen = kph;
					}
					else if(recov_time[i] <= 90 && recov_time[i] > 45)
					{
						ik(out, 0, -0.11);
						kp_chosen = kph;
					} 
					else if(recov_time[i] <= 150 && recov_time[i] > 90)
					{
						ik(out, -0.05, -0.18);
						kp_chosen = kpl;
					} 
					else
					{
						hit[i] = false;
						recov_time[i] = 0;
					}

					if(hit[i])
					{
						if(i==0 || i==1)  // smaller is qout
						{
							pwm[i*2] = kp_chosen*(out[1]-joint[i*2].getPosition())+kd*(0-joint[i*2].getVelocity());
							pwm[i*2+1]= kp_chosen*(out[0]-joint[i*2+1].getPosition())+kd*(0-joint[i*2+1].getVelocity());
						}
						else
						{
							pwm[i*2] = kp_chosen*(out[0]-joint[i*2].getPosition())+kd*(0-joint[i*2].getVelocity());
							pwm[i*2+1]= kp_chosen*(out[1]-joint[i*2+1].getPosition())+kd*(0-joint[i*2+1].getVelocity());						
						}
					}
				}

				if(miss[i])
				{
					recov_time[i]++;
					float out[2];
					float kp_chosen;
					if(recov_time[i] <= 100)
					{
						ik(out, -0.02, -0.25);  // backwards a bit
						kp_chosen = kpl;
					}
					else if(recov_time[i] <= 300 && recov_time[i] > 100)
					{
						ik(out, 0.02, -0.22);
						kp_chosen = kpm;
					} 
					// else if(recov_time[i] <= 150 && recov_time[i] > 90)
					// {
					// 	ik(out, -0.05, -0.18);
					// 	kp_chosen = kpl;
					// } 
					else
					{
						miss[i] = false;
						recov_time[i] = 0;
					}

					if(miss[i])
					{
						if(i==0 || i==1)  // smaller is qout
						{
							pwm[i*2] = kp_chosen*(out[1]-joint[i*2].getPosition())+kd*(0-joint[i*2].getVelocity());
							pwm[i*2+1]= kp_chosen*(out[0]-joint[i*2+1].getPosition())+kd*(0-joint[i*2+1].getVelocity());
						}
						else
						{
							pwm[i*2] = kp_chosen*(out[0]-joint[i*2].getPosition())+kd*(0-joint[i*2].getVelocity());
							pwm[i*2+1]= kp_chosen*(out[1]-joint[i*2+1].getPosition())+kd*(0-joint[i*2+1].getVelocity());						
						}
					}
				}
			}

			for(int i=0; i<8; i++)
			{
				if(!hit[i/2] && !miss[i/2])
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
