/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De and Turner Topping <avik@ghostrobotics.io> <turner@ghostrobotics.io>
 * 
 * Zhiyi (Allen) Ren
 * FirstHop example with touchdown detection using leg observer
 * 
 */
#include <stdio.h>
#include <SDK.h>
#include <math.h>
#include <Motor.h>
#include <ReorientableBehavior.h>

#include <string>
#include <iostream>

#include "func.h"
#include "observer.h"

#if defined(ROBOT_MINITAUR)
// Subject to change for individual robots
// const float motZeros[8] = {2.82, 3.435, 3.54, 3.076, 1.03, 3.08, 6.190, 1.493};
const float motZeros[8] = {2.570, 3.167, 3.777, 3.853, 2.183, 1.556, .675, 2.679}; // RML Ellie
// const float motZeros[8] = {0.631, 4.076, 1.852, 3.414, 1.817, 5.500, 1.078, 6.252}; //RML Odie
#endif

// State machine representation of behavior
enum FHMode
{
	FH_SIT = 0,
	FH_STAND,
	FH_LEAP,
	FH_LAND
};

char myData[32];
float *myData_buf = (float*)myData;

// float pos_act[2] = {0,0};
// float vel_act[2] = {0,0};
// float pos_obs_int[2] = {0,0};
// float vel_obs_int[2] = {0,0};
// float pos_obs[2] = {0,0};
// float vel_obs[2] = {0,0};

float force_est[4] = {0,0,0,0};
float batt_act = 0;

int count = 0;
int landTime = 0;

float pos_act[8] = {0,0,0,0,0,0,0,0};
float vel_act[8] = {0,0,0,0,0,0,0,0};
float pos_obs_int[8] = {0,0,0,0,0,0,0,0};
float vel_obs_int[8] = {0,0,0,0,0,0,0,0};
float pos_obs[8] = {0,0,0,0,0,0,0,0};
float vel_obs[8] = {0,0,0,0,0,0,0,0};

float res_pos[8] = {0,0,0,0,0,0,0,0};
float res_vel[8] = {0,0,0,0,0,0,0,0};
float res_pos_avg = 0;
float res_vel_avg = 0;

int lastTime = 0;
int thisTime = 0;

class ObsHop : public ReorientableBehavior
{
public:
	FHMode mode = FH_SIT; //Current state within state-machine

	uint32_t tLast; //int used to store system time at various events

	float lastExtension; //float for storing leg extension during the last control loop
	float exCmd;				 //The commanded leg extension
	float extDes;				 //The desired leg extension
	float angDes;				 // The desired leg angle

	bool unitUpdated;

	const float maxDeltaExtCmd = 0.002;
	const float kExtAnimRate = 0.002;

	int t;
	bool start = true;
	int tStart;
	int standCount = 0;

	// observer stuff
	float pwm[8];
	float integral[8];

	void signal(uint32_t sig)
	{
		if(sig > 1)
			mode = FH_LEAP;
	}

	void begin()
	{
		ioctl(LOGGER_FILENO, 0); // do not log

		mode = FH_STAND;			// Start behavior in STAND mode
		tLast = S->millis;		// Record the system time @ this transition
		exCmd = 0.14;					// Set extension command to be 0.14m, the current value of extension
		lastExtension = 0.14; // Record the previous loop's extension; it should be 0.14m
	}

	void update()
	{
		if (isReorienting())
			return;
		C->mode = RobotCommand_Mode_LIMB;
		if (mode == FH_SIT)
		{
			ioctl(LOGGER_FILENO, 0); // do not log

			for (int i = 0; i < P->limbs_count; ++i)
			{
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;
				angDes = (isFront(i)) ? -S->imu.euler.y - 0.1 : -S->imu.euler.y + 0.2;
				limb[i].setGain(ANGLE, 0.8, .03);
				limb[i].setPosition(ANGLE, angDes);

				limb[i].setGain(EXTENSION, 120, 3);
				limb[i].setPosition(EXTENSION, 0.14);
			}
		}
		else if (mode == FH_STAND)
		{
			if(start || standCount >= 100)
			{
				ioctl(LOGGER_FILENO, 0); // do not log
			}
			else
			{
				ioctl(LOGGER_FILENO, 1); // log
			}

			extDes = map(C->behavior.pose.position.z, -1.0, 1.0, 0.14, 0.25);
			if (S->millis - tLast < 250 && exCmd < extDes)
			{
				exCmd = exCmd + (extDes - lastExtension) * kExtAnimRate;
			}
			else
			{
				if (extDes - exCmd > maxDeltaExtCmd)
				{
					exCmd = exCmd + maxDeltaExtCmd;
				}
				else if (exCmd - extDes > maxDeltaExtCmd)
				{
					exCmd = exCmd - maxDeltaExtCmd;
				}
				else
				{
					exCmd = extDes;
				}
			}
			for (int i = 0; i < P->limbs_count; ++i)
			{
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;
				// Leg splay
				angDes = (isFront(i)) ? -S->imu.euler.y - 0.1 : -S->imu.euler.y + 0.2;
				limb[i].setGain(ANGLE, 0.8 + 0.2 * ((extDes - 0.12) / 0.13), 0.03);
				limb[i].setPosition(ANGLE, angDes);

				limb[i].setGain(EXTENSION, 120, 4);
				limb[i].setPosition(EXTENSION, exCmd);
			}

			if(!start && standCount < 100)  // keep observer for 100 cycles
			{
				standCount++;
				batt_act = S->batt.voltage;
				// copy over
				for(int i=0; i<8; i++)
				{
					pos_obs[i] = pos_obs_int[i];
					vel_obs[i] = vel_obs_int[i];
				}
				// record actual values
				pos_act[0] = joint[1].getPosition();
				pos_act[1] = joint[0].getPosition();
				force_est[0] = limb[0].getForce(EXTENSION);
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

				// calculate residual, int has the current value, obs has future value from previous step
				res_pos[0] = pos_obs[0] - pos_obs_int[0];
				res_pos[1] = pos_obs[1] - pos_obs_int[1];
				res_vel[0] = vel_obs[0] - vel_obs_int[0];
				res_vel[1] = vel_obs[1] - vel_obs_int[1];
				res_pos_avg = (abs(res_pos[0])+abs(res_pos[1]))*0.5;
				res_vel_avg = (abs(res_vel[0])+abs(res_vel[1]))*0.5;

				for(int i=0; i<4; i++)
				{
					float PWM1, PWM2;
					if(i == 0)
					{
						PWM1 = joint[1].getOpenLoop();
						PWM2 = joint[0].getOpenLoop();
					}
					else if(i==1)
					{
						PWM1 = joint[3].getOpenLoop();
						PWM2 = joint[2].getOpenLoop();					
					}
					else if(i==2)
					{
						PWM1 = joint[4].getOpenLoop();
						PWM2 = joint[5].getOpenLoop();
					}
					else
					{
						PWM1 = joint[6].getOpenLoop();
						PWM2 = joint[7].getOpenLoop();					
					}
					obs(pos_obs_int, vel_obs_int, batt_act, PWM1, PWM2, i);
				}
			}
		}
		else if (mode == FH_LEAP)
		{
			ioctl(LOGGER_FILENO, 0); // do not log
			for (int i = 0; i < P->limbs_count; ++i)
			{
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
				limb[i].setOpenLoop(EXTENSION, 1);
				limb[i].setGain(ANGLE, 1.0, 0.03);
				limb[i].setPosition(ANGLE, -S->imu.euler.y);
				if (limb[i].getPosition(EXTENSION) > 2.7)
				{
					mode = FH_LAND;
					tLast = S->millis;
					unitUpdated = false;
				}
			}

			// copy initial conditions for landing
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
		}
		else if (mode == FH_LAND)
		{
			ioctl(LOGGER_FILENO, 1); // start logging
			start = false;

			// t  = (S->millis - tStart) % 5;  // correct the starting time
			// if(t == 0)
			// {
				batt_act = S->batt.voltage;
				for(int i=0; i<8; i++)
				{
					pos_obs[i] = pos_obs_int[i];
					vel_obs[i] = vel_obs_int[i];
				}

				// record actual values
				pos_act[0] = joint[1].getPosition();
				pos_act[1] = joint[0].getPosition();
				force_est[0] = limb[0].getForce(EXTENSION);

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

				// calculate residual, int has the current value, obs has future value from previous step
				res_pos[0] = pos_obs[0] - pos_obs_int[0];
				res_pos[1] = pos_obs[1] - pos_obs_int[1];
				res_vel[0] = vel_obs[0] - vel_obs_int[0];
				res_vel[1] = vel_obs[1] - vel_obs_int[1];
				res_pos_avg = (abs(res_pos[0])+abs(res_pos[1]))*0.5;
				res_vel_avg = (abs(res_vel[0])+abs(res_vel[1]))*0.5;
 			// }

			for (int i = 0; i < P->limbs_count; ++i)
			{
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;
				exCmd = 0.25;
				angDes = (isFront(i)) ? -S->imu.euler.y - 0.1 : -S->imu.euler.y + 0.2;

				limb[i].setGain(ANGLE, 1.2, 0.03);
				limb[i].setPosition(ANGLE, angDes);

				limb[i].setGain(EXTENSION, 150, 5);
				limb[i].setPosition(EXTENSION, exCmd);

				float PWM1, PWM2;
				if(i == 0)
				{
					PWM1 = joint[1].getOpenLoop();
					PWM2 = joint[0].getOpenLoop();
				}
				else if(i==1)
				{
					PWM1 = joint[3].getOpenLoop();
					PWM2 = joint[2].getOpenLoop();					
				}
				else if(i==2)
				{
					PWM1 = joint[4].getOpenLoop();
					PWM2 = joint[5].getOpenLoop();
				}
				else
				{
					PWM1 = joint[6].getOpenLoop();
					PWM2 = joint[7].getOpenLoop();					
				}

				// if(i == 0) obs(pos_obs_int, vel_obs_int, batt_act, PWM1, PWM2, i);

				// this takes almost 4ms
				obs(pos_obs_int, vel_obs_int, batt_act, PWM1, PWM2, i);

				// TODO: use observer instead here for detection
				if (limb[i].getForce(EXTENSION) > 40 && S->millis - tLast > 20)
				{
					mode = FH_STAND;
					tLast = S->millis;
					exCmd = 0.25;
					lastExtension = 0.25;
				}
			}
			// if(lastTime == 0)
			// {
			// 	lastTime = S->millis;
			// 	return;
			// }
			// if(thisTime == 0)
			// {
			// 	thisTime = S->millis;
			// 	return;
			// }

			// count++;
			// landTime = S->millis - tStart;
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
	myData_buf[0] = toQ1(pos_obs[0]);
	myData_buf[1] = toQ2(pos_obs[1]);
	myData_buf[2] = pos_act[0];
	myData_buf[3] = pos_act[1];
	myData_buf[4] = res_pos_avg;
	myData_buf[5] = res_vel_avg;
	myData_buf[6] = force_est[0];
	// myData_buf[7];

	write(LOGGER_FILENO, myData, 32);
}

int main(int argc, char *argv[])
{
	init(RobotParams_Type_MINITAUR, argc, argv);
	for (int i = 0; i < P->joints_count; ++i)
		P->joints[i].zero = motZeros[i]; //Add motor zeros from array at beginning of file

  setDebugRate(100);

	// Declare instance of our behavior
	ObsHop obsHop;

	// Add our behavior to the behavior vector (Walk and Bound are already there)
	behaviors.push_back(&obsHop);
	ioctl(LOGGER_FILENO, 0); // do not log
	return begin();
}
