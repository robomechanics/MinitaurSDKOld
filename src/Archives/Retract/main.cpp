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

bool done = false;

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

// State machine representation of behavior
enum FHMode
{
	FH_SIT = 0,
	FH_STAND,
	FH_FORWARD, FH_FORWARD_2,
};

class Retract : public ReorientableBehavior
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

	// use joystick to start walking or stop
	void signal(uint32_t sig)
	{
		if (sig == 3)
		{
			mode = FH_FORWARD;
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

		ioctl(LOGGER_FILENO, 0); // do not log
	}

	void update()
	{
		if (isReorienting())
			return;
		C->mode = RobotCommand_Mode_LIMB;

		if(S->millis > oldMillis + 400)
		{
			timer++;
			oldMillis = S->millis;
		}

		if (mode == FH_SIT)
		{
			ioctl(LOGGER_FILENO, 0); // do not log

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
			ioctl(LOGGER_FILENO, 0); // do not log

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

		else if (mode == FH_FORWARD)
		{ 
			// if(!done)
			// {
				ioctl(LOGGER_FILENO, 1); // start logging
			// }

			// Leg 0
			P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
			limb[0].setGain(ANGLE, 0.6, 0.01);
			limb[0].setPosition(ANGLE, 0);
			limb[0].setGain(EXTENSION, 120, 4);
			limb[0].setPosition(EXTENSION, 0.12);
			// Leg 3
			P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
			limb[3].setGain(ANGLE, 0.6, 0.01);
			limb[3].setPosition(ANGLE, 0);
			limb[3].setGain(EXTENSION, 120, 4);
			limb[3].setPosition(EXTENSION, 0.12);

			// Leg 1
			P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
			limb[1].setGain(ANGLE, 0.6, 0.01);
			limb[1].setPosition(ANGLE, 0);
			limb[1].setGain(EXTENSION, 120, 4);
			limb[1].setPosition(EXTENSION, 0.12);
			// Leg 2
			P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
			limb[2].setGain(ANGLE, 0.6, 0.01);
			limb[2].setPosition(ANGLE, 0);
			limb[2].setGain(EXTENSION, 120, 4);
			limb[2].setPosition(EXTENSION, 0.12);

			if (timer %2 != 0){
				mode = FH_FORWARD_2;
			}
		}

		else if (mode == FH_FORWARD_2)
		{
			// Leg 0
			P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
			limb[0].setGain(ANGLE, 0.8, 0.01);
			limb[0].setPosition(ANGLE, 0);
			limb[0].setGain(EXTENSION, 120, 4);
			limb[0].setPosition(EXTENSION, 0.20);
			// Leg 3
			P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
			limb[3].setGain(ANGLE, 0.8, 0.01);
			limb[3].setPosition(ANGLE, 0);
			limb[3].setGain(EXTENSION, 120, 4);
			limb[3].setPosition(EXTENSION, 0.20);

			// Leg 1
			P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
			limb[1].setGain(ANGLE, 0.8, 0.01);
			limb[1].setPosition(ANGLE, 0);
			limb[1].setGain(EXTENSION, 120, 4);
			limb[1].setPosition(EXTENSION, 0.20);
			// Leg 2
			P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
			limb[2].setGain(ANGLE, 0.8, 0.01);
			limb[2].setPosition(ANGLE, 0);
			limb[2].setGain(EXTENSION, 120, 4);
			limb[2].setPosition(EXTENSION, 0.20);

			if (timer %2 == 0){
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
	// double t = S->millis;
	// if(t > 10000)  // 10 seconds
	// {
	// 	std::cout << end << "\n";
	// 	// ioctl(LOGGER_FILENO, 0); // stop logging
	// 	// done = true;
	// 	return;
	// }

	myData_buf[0] = joint[1].getOpenLoop();
	myData_buf[1] = joint[0].getOpenLoop();
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

	//Disable the safety shut off feature:
	//IT IS POSSIBLE TO DAMAGE THE MOTOR; BE CAREFUL WHEN UfastsinG
	//BEHAVIORS WITHOUT THIS FAILSAFE 
	// safetyShutoffEnable(false);
	//Disable the softStart feature
	// softStartEnable(false);

	Retract retract;								 // Declare instance of our behavior
	behaviors.push_back(&retract);

	return begin();
}
