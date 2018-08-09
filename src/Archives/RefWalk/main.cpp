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
#include <VirtualLeg.h>

#include <string>
#include <iostream>

std::string end("end");

#if defined(ROBOT_MINITAUR)
// Subject to change for individual robots
const float motZeros[8] = {2.570, 3.167, 3.777, 3.853, 2.183, 1.556, .675, 2.679}; // RML Ellie
// const float motZeros[8] = {0.631, 4.076, 1.852, 3.414, 1.817, 5.500, 1.078, 6.252}; //RML Odie
#endif

//Maximum difference between commanded and actual leg extension
const float maxDeltaExtCmd = 0.002;
const float kExtAnimRate = 0.002; //Maximum rate (in m/s) of change in cmdExt

char myData[32];
float *myData_buf = (float*)myData;

const float apExt = 0.11;
const float tdExt = 0.17;
const float liExt = 0.17;
const float boExt = 0.18;

const float kExtPStance = 120, kExtDStance = 4;
const float kExtPFlight = 100, kExtDFlight = 4;
const float kAngPStance = 1.0, kAngDStance = 0.03;
const float kAngPFlight = 0.8, kAngDFlight = 0.03;

const float forAngle = -20;
const float bacAngle = 20;
const float forRad = forAngle*3.14/180;
const float bacRad = bacAngle*3.14/180;

float leftAngles[] = {0, bacRad, 0, forRad, 0};
float rightAngles[] = {0, forRad, 0, bacRad, 0};
float leftExtensions[] = {boExt, liExt, apExt, tdExt, boExt};
float rightExtensions[] = {apExt, tdExt, boExt, liExt, apExt};
// float leftExtVels[] = {0,-0.001,0,0.001,0};
// float leftAngVels[] = {bacRad/200,0,forRad/200,0,bacRad/200};
// float rightExtVels[] = {0,0.001,0,-0.001,0};
// float rightAngVels[] = {forRad/200,0,bacRad/200,0,forRad/200};
float leftExtVels[] = {0,0,0,0,0};
float leftAngVels[] = {0,0,0,0,0};
float rightExtVels[] = {0,0,0,0,0};
float rightAngVels[] = {0,0,0,0,0};

// float leftTimes[] = {0,200,400,600,800};
// float rightTimes[] = {0,200,400,600,800};
float leftTimes[] = {0,125,250,375,500};
float rightTimes[] = {0,125,250,375,500};

int numpoints = (sizeof leftTimes)/(sizeof rightTimes[0]);
int period = leftTimes[4];

// State machine representation of behavior
enum FHMode
{
	FH_SIT = 0,
	FH_STAND,
	FH_FORWARD
};

class RefWalk : public ReorientableBehavior
{
public:
	FHMode mode = FH_SIT; // Current state within state-machine

	uint32_t tLast;       // int used to store system time at various events

	float lastExtension;  // float for storing leg extension during the last control loop
	float exCmd;				  // The commanded leg extension
	float extDes;				  // The desired leg extension
	float angDes;				  // The desired leg angle

	int count = 0;
	int curTime;
	int oldTime;

	bool leftStance = true;
	bool rightStance = true;

	Interpolator leftInterp = Interpolator(numpoints);
	Interpolator rightInterp = Interpolator(numpoints);

	float ang[2], ext[2];  // left, right
	bool ifLeft[4] = {true, false, false, true};
	int t;

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

		oldTime = S->millis;
		ioctl(LOGGER_FILENO, 0); // do not logging
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
				// limb[i].setPosition(ANGLE, angDes);
				limb[i].setPosition(ANGLE, 0);

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
				// limb[i].setPosition(ANGLE, angDes);
				limb[i].setPosition(ANGLE, 0);

				limb[i].setGain(EXTENSION, 120, 4);
				limb[i].setPosition(EXTENSION, exCmd);
			}
		}

		else if (mode == FH_FORWARD)
		{
			ioctl(LOGGER_FILENO, 1); // start logging

			t = S->millis % period;

			ang[0] = leftInterp.getfastsinglePVTInterp(leftAngles, leftAngVels, leftTimes, t);
			ext[0] = leftInterp.getfastsinglePVTInterp(leftExtensions, leftExtVels, leftTimes, t);
			ang[1] = rightInterp.getfastsinglePVTInterp(rightAngles, rightAngVels, rightTimes, t);
			ext[1] = rightInterp.getfastsinglePVTInterp(rightExtensions, rightExtVels, rightTimes, t);

			for(int i = 0; i < 4; i++)
			{
				limb[i].setGain(ANGLE, 1.0, 0.02);
				limb[i].setGain(EXTENSION, 140, 4);

				if(ifLeft[i])
				{
					limb[i].setPosition(ANGLE, ang[0]);
					limb[i].setPosition(EXTENSION, ext[0]);
				}
				else
				{
					limb[i].setPosition(ANGLE, ang[1]);
					limb[i].setPosition(EXTENSION, ext[1]);					
				}
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
	double t = S->millis;
	if(t > 10000)  // 10 seconds
	{
		// std::cout << end << "\n";
		ioctl(LOGGER_FILENO, 0); // stop logging
		return;
	}

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

	RefWalk refWalk;								 // Declare instance of our behavior
	behaviors.push_back(&refWalk);

	return begin();
}

// const float leftExtKp[10] = {kExtPStance, kExtPStance, kExtPStance, kExtPStance, kExtPStance, 																	kExtPStance, kExtPFlight, kExtPFlight, kExtPFlight, kExtPFlight};
// const float rightExtKp[10] = {kExtPStance, kExtPFlight, kExtPFlight, kExtPFlight, kExtPFlight, 																kExtPStance, kExtPStance, kExtPStance, kExtPStance, kExtPStance};

// const float leftExtKd[10] = {kExtDStance, kExtDStance, kExtDStance, kExtDStance, kExtDStance,
// 													   kExtDStance, kExtDFlight, kExtDFlight, kExtDFlight, kExtDFlight};
// const float rightExtKd[10] = {kExtDStance, kExtDFlight, kExtDFlight, kExtDFlight, kExtDFlight,
// 															kExtDStance, kExtDStance, kExtDStance, kExtDStance, kExtDStance};

// const float leftAngKp[10] = {kAngPStance, kAngPStance, kAngPStance, kAngPStance, kAngPStance,
// 													   kAngPStance, kAngPFlight, kAngPFlight, kAngPFlight, kAngPFlight};
// const float rightAngKp[10] = {kAngPStance, kAngPFlight, kAngPFlight, kAngPFlight, kAngPFlight,
// 															kAngPStance, kAngPStance, kAngPStance, kAngPStance, kAngPStance};

// const float leftAngKd[10] = {kAngDStance, kAngDStance, kAngDStance, kAngDStance, kAngDStance,
// 													   kAngDStance, kAngDFlight, kAngDFlight, kAngDFlight, kAngDFlight};
// const float rightAngKd[10] = {kAngDStance, kAngDFlight, kAngDFlight, kAngDFlight, kAngDFlight,
// 															kAngDStance, kAngDStance, kAngDStance, kAngDStance, kAngDStance};

			// P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
			// P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
			// P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
			// P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;

			// curTime = S->millis;
			// if(curTime - oldTime > 50)
			// {
			// 	count++;
			// 	if(count > 9) count = 0;

			// 	oldTime = curTime;
			// }

			// // left
			// limb[0].setGain(ANGLE, leftAngKp[count], leftAngKd[count]);
			// limb[0].setPosition(ANGLE, leftAngleAll[count]);
			// limb[0].setGain(EXTENSION, leftExtKp[count], leftExtKd[count]);
			// limb[0].setPosition(EXTENSION, leftExtAll[count]);
			// limb[3].setGain(ANGLE, leftAngKp[count], leftAngKd[count]);
			// limb[3].setPosition(ANGLE, leftAngleAll[count]);
			// limb[3].setGain(EXTENSION, leftExtKp[count], leftExtKd[count]);
			// limb[3].setPosition(EXTENSION, leftExtAll[count]);

			// // right
			// limb[1].setGain(ANGLE, rightAngKp[count], rightAngKd[count]);
			// limb[1].setPosition(ANGLE, rightAngleAll[count]);
			// limb[1].setGain(EXTENSION, rightExtKp[count], rightExtKd[count]);
			// limb[1].setPosition(EXTENSION, rightExtAll[count]);
			// limb[2].setGain(ANGLE, rightAngKp[count], rightAngKd[count]);
			// limb[2].setPosition(ANGLE, rightAngleAll[count]);
			// limb[2].setGain(EXTENSION, rightExtKp[count], rightExtKd[count]);
			// limb[2].setPosition(EXTENSION, rightExtAll[count]);


// LegPair left(0,3);
// LegPair right(1,2);

// const float leftExtAll[10] = {boExt, boExt, boExt, liExt, liExt, liExt, apExt, apExt, tdExt, tdExt};
// const float leftAngleAll[10] = {0, 0, 0, bacRad, bacRad, bacRad, 0, 0, forRad, forRad};

// const float rightExtAll[10] = {liExt, apExt, apExt, tdExt, tdExt, boExt, boExt, boExt, liExt, liExt};
// const float rightAngleAll[10] = {bacRad, 0, 0, forRad, forRad, 0, 0, 0, bacRad, bacRad};


// const int tflight = 170, tminstance = 100;