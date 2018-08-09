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

#include <string>
#include <iostream>

std::string end("end");

#if defined(ROBOT_MINITAUR)
// Subject to change for individual robots
// const float motZeros[8] = {2.82, 3.435, 3.54, 3.076, 1.03, 3.08, 6.190, 1.493};
const float motZeros[8] = {2.570, 3.167, 3.777, 3.853, 2.183, 1.556, .675, 2.679}; // RML Ellie
// const float motZeros[8] = {0.631, 4.076, 1.852, 3.414, 1.817, 5.500, 1.078, 6.252}; //RML Odie
#endif

//Maximum difference between commanded and actual leg extension
const float maxDeltaExtCmd = 0.002;
const float kExtAnimRate = 0.002; //Maximum rate (in m/s) of change in cmdExt

char myData[32];
float *myData_buf = (float*)myData;

// typedef struct OutStruct {
//     // uint32_t millis;
// 		// float position;
// 		// float torqueEst;
// 		float PWM;
// 		// float voltage;
// } __attribute__((packed)) OutStruct;
// OutStruct outs;

// State machine representation of behavior
enum FHMode
{
	FH_SIT = 0,
	FH_STAND,
	FH_START, FH_FORWARD, FH_UP, FH_LAND
};

class Silly : public ReorientableBehavior
{
public:
	FHMode mode = FH_SIT; // Current state within state-machine

	uint32_t tLast;       // int used to store system time at various events

	float lastExtension;  // float for storing leg extension during the last control loop
	float exCmd;				  // The commanded leg extension
	float extDes;				  // The desired leg extension
	float angDes;				  // The desired leg angle
	int timer;
	int oldMillis = S->millis;
	int curMillis;

	// use joystick to start walking or stop
	void signal(uint32_t sig)
	{
		if (sig == 3)
		{
			mode = FH_START;
		}
		else
		{
			mode = FH_SIT;
		}
	}

	void begin()
	{
		mode = FH_STAND;			// Start behavior in STAND mode
		tLast = S->millis;		// Record the system time @ this transition
		exCmd = 0.14;				  // Set extension command to be 0.14m, the current value of extension
		lastExtension = 0.14; // Record the previous loop's extension; it should be 0.14m

		P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
		P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
		P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
		P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
	}

	void update()
	{
		if (isReorienting())
			return;
		C->mode = RobotCommand_Mode_LIMB;

		if(S->millis > oldMillis + 400)
		{
			timer++;
			// oldMillis = S->millis;
		}

		if (mode == FH_SIT)
		{
			for (int i = 0; i < P->limbs_count; ++i)
			{
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;

				angDes = (isFront(i)) ? -S->imu.euler.y - 0.1 : -S->imu.euler.y + 0.2;
				limb[i].setGain(ANGLE, 0.8, .03);
				limb[i].setPosition(ANGLE, angDes);
				// limb[i].setPosition(ANGLE, 0);

				limb[i].setGain(EXTENSION, 120, 3);
				limb[i].setPosition(EXTENSION, 0.14);
			}
		}

		else if (mode == FH_STAND)
		{
			extDes = map(C->behavior.pose.position.z, -1.0, 1.0, 0.11, 0.25);
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

				angDes = (isFront(i)) ? -S->imu.euler.y - 0.1 : -S->imu.euler.y + 0.2;
				// Stiffen the angle gain linearly as a function of the extension
				// This way, more torque is provided as the moment arm becomes longer.
				limb[i].setGain(ANGLE, 0.8 + 0.2 * ((extDes - 0.12) / 0.13), 0.03);
				limb[i].setPosition(ANGLE, angDes);

				limb[i].setGain(EXTENSION, 120, 4);
				limb[i].setPosition(EXTENSION, exCmd);
			}
		}

		else if (mode == FH_START)
		{
			oldMillis = S->millis;
			mode = FH_FORWARD;
		}

		else if (mode == FH_FORWARD)
		{
			// support
			limb[0].setGain(ANGLE, 1.5, 0.03);
			limb[0].setPosition(ANGLE, -S->imu.euler.y-0.1);
			limb[0].setGain(EXTENSION, 120, 4);
			limb[0].setPosition(EXTENSION, 0.25);

			limb[2].setGain(ANGLE, 1.5, 0.03);
			limb[2].setPosition(ANGLE, -S->imu.euler.y-0.1);
			limb[2].setGain(EXTENSION, 120, 4);
			limb[2].setPosition(EXTENSION, 0.25);

			limb[1].setGain(ANGLE, 1.5, 0.03);
			limb[1].setPosition(ANGLE, -S->imu.euler.y+0.1);
			limb[1].setGain(EXTENSION, 120, 4);
			limb[1].setPosition(EXTENSION, 0.12);

			limb[3].setGain(ANGLE, 1.5, 0.03);
			limb[3].setPosition(ANGLE, -S->imu.euler.y+0.1);
			limb[3].setGain(EXTENSION, 120, 4);
			limb[3].setPosition(EXTENSION, 0.12);

			curMillis = S->millis;
			if(curMillis - oldMillis > 1000)
			{
				mode = FH_UP;
				oldMillis = S->millis;
			}
		}

		else if (mode == FH_UP)
		{
			// P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
			// P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_RAD;

			limb[1].setOpenLoop(EXTENSION, 3);
			limb[3].setOpenLoop(EXTENSION, 3);
			limb[1].setGain(ANGLE, 1.0, 0.03);
			limb[1].setGain(ANGLE, 1.0, 0.03);
			limb[1].setPosition(ANGLE, -S->imu.euler.y+0.1);
			limb[3].setPosition(ANGLE, -S->imu.euler.y+0.1);

			// CHANGE REAR GAIN HERE?
			limb[0].setGain(EXTENSION, 0.4, 0.01);
			limb[2].setGain(EXTENSION, 0.4, 0.01);
			limb[0].setPosition(EXTENSION, 0.12);
			limb[2].setPosition(EXTENSION, 0.12);
			limb[0].setGain(ANGLE, 1.0, 0.03);
			limb[2].setGain(ANGLE, 1.0, 0.03);
			limb[0].setPosition(ANGLE, -S->imu.euler.y-0.1);
			limb[2].setPosition(ANGLE, -S->imu.euler.y-0.1);

			if(limb[1].getPosition(EXTENSION) > 0.27)
			{
				mode = FH_LAND;
				oldMillis = S->millis;
			}
		}

		else if (mode == FH_LAND)
		{
			P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
			P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;

			limb[1].setGain(ANGLE, 0.6, 0.01);
			limb[1].setPosition(ANGLE, 0);
			limb[1].setGain(EXTENSION, 120, 4);
			limb[1].setPosition(EXTENSION, 0.15);

			limb[3].setGain(ANGLE, 0.6, 0.01);
			limb[3].setPosition(ANGLE, 0);
			limb[3].setGain(EXTENSION, 120, 4);
			limb[3].setPosition(EXTENSION, 0.15);

			curMillis = S->millis;
			if(curMillis - oldMillis > 1000)
			{
				mode = FH_FORWARD;
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
}

int main(int argc, char *argv[])
{
	// MINITAUR
	init(RobotParams_Type_MINITAUR, argc, argv);
	for (int i = 0; i < P->joints_count; ++i)
		P->joints[i].zero = motZeros[i];

	Silly silly;
	behaviors.push_back(&silly);

	return begin();
}
