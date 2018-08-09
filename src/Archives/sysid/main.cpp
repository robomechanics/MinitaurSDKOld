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

#include <string>
#include <iostream>

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

const float apExt = 0.12;
const float tdExt = 0.18;
const float liExt = 0.18;
const float boExt = 0.19;

const float forAngle = -20;
const float bacAngle = 20;
const float forRad = forAngle*3.14/180;
const float bacRad = bacAngle*3.14/180;

float angles[] = {0, bacRad, 0, forRad, 0};
float extensions[] = {boExt, liExt, apExt, tdExt, boExt};
float angVels[] = {0,0,0,0,0};
float extVels[] = {0,0,0,0,0};

float times[] = {0,125,250,375,500};

float numpoints = (sizeof times)/(sizeof times[0]);
int period = times[4];

Interpolator interp = Interpolator(numpoints);


// State machine representation of behavior
enum FHMode
{
	FH_SIT = 0,
	FH_STAND,
	FH_START
};

class Sysid : public ReorientableBehavior
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

	float ang, ext;  // left, right
	int t;

	// for one motor testing
	float x,y;  // toe in cartesian space
	float q1, q2;  // motor angles, in, out

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

		// P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
		// P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
		// P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
		// P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
	}

	void update()
	{
		// if (isReorienting())
		// 	return;
		// C->mode = RobotCommand_Mode_LIMB;
		ioctl(LOGGER_FILENO, 0); // start logging
		C->mode = RobotCommand_Mode_JOINT;


		if(S->millis > oldMillis + 400)
		{
			timer++;
			// oldMillis = S->millis;
		}

		if (mode == FH_SIT)
		{
		// 	ioctl(LOGGER_FILENO, 0); // start logging

		// 	for (int i = 0; i < P->limbs_count; ++i)
		// 	{
		// 		P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;

		// 		angDes = (isFront(i)) ? -S->imu.euler.y - 0.1 : -S->imu.euler.y + 0.2;
		// 		limb[i].setGain(ANGLE, 0.8, .03);
		// 		limb[i].setPosition(ANGLE, angDes);
		// 		// limb[i].setPosition(ANGLE, 0);

		// 		limb[i].setGain(EXTENSION, 120, 3);
		// 		limb[i].setPosition(EXTENSION, 0.14);
		// 	}

		  joint[0].setGain(0.6,0.01);
			joint[1].setGain(0.6,0.01);
			
			joint[0].setPosition(3*3.14/2-2.98);  // outer
			joint[1].setPosition(0.16+3.14/2);  // inner
		}

		// else if (mode == FH_STAND)
		// {
		// 	ioctl(LOGGER_FILENO, 0); // start logging			

		// 	extDes = map(C->behavior.pose.position.z, -1.0, 1.0, 0.11, 0.25);
		// 	if (S->millis - tLast < 250 && exCmd < extDes)
		// 	{
		// 		exCmd = exCmd + (extDes - lastExtension) * kExtAnimRate;
		// 	}
		// 	else
		// 	{
		// 		if (extDes - exCmd > maxDeltaExtCmd)
		// 		{
		// 			exCmd = exCmd + maxDeltaExtCmd;
		// 		}
		// 		else if (exCmd - extDes > maxDeltaExtCmd)
		// 		{
		// 			exCmd = exCmd - maxDeltaExtCmd;
		// 		}
		// 		else
		// 		{
		// 			exCmd = extDes;
		// 		}
		// 	}
		// 	for (int i = 0; i < P->limbs_count; ++i)
		// 	{
		// 		P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;

		// 		angDes = (isFront(i)) ? -S->imu.euler.y - 0.1 : -S->imu.euler.y + 0.2;
		// 		// Stiffen the angle gain linearly as a function of the extension
		// 		// This way, more torque is provided as the moment arm becomes longer.
		// 		limb[i].setGain(ANGLE, 0.8 + 0.2 * ((extDes - 0.12) / 0.13), 0.03);
		// 		limb[i].setPosition(ANGLE, angDes);

		// 		limb[i].setGain(EXTENSION, 120, 4);
		// 		limb[i].setPosition(EXTENSION, exCmd);
		// 	}
		// }

		if (mode == FH_START)
		{
			ioctl(LOGGER_FILENO, 1); // start logging

			t = S->millis % period;

			ang = interp.getfastsinglePVTInterp(angles, angVels, times, t);
			ext = interp.getfastsinglePVTInterp(extensions, extVels, times, t);

			// convert to cartesian coords, then ik to get motor angles
			x = ext*fastsin(-ang);
			y = -ext*fastcos(-ang);

			float r = fastsqrt(pow(x,2)+pow(y,2));
			float theta = -atan2(y,x)-3.14/2;
			float diffAng = 3.14-acos((-0.03+pow(r,2))/(0.2*r));

			q1 = diffAng+theta;
			q2 = diffAng-theta;

			// C->mode = RobotCommand_Mode_JOINT;

			joint[0].setGain(0.6,0.01);
			joint[1].setGain(0.6,0.01);
			
			joint[0].setPosition(q2);  // outer
			joint[1].setPosition(q1);  // inner
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

	Sysid sysid;
	behaviors.push_back(&sysid);

	return begin();
}
