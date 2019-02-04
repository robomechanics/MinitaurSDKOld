/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De and Turner Topping <avik@ghostrobotics.io> <turner@ghostrobotics.io>
 */
#include <stdio.h>
#include <SDK.h>
#include <math.h>
#include <Motor.h>
#include <ReorientableBehavior.h>
#include "MinitaurShinContact.h"
#include "minitaurVelocity.h"
using namespace std;

#define arraySize 500

#if defined(ROBOT_MINITAUR)
// Subject to change for individual robots
// const float motZeros[8] = {2.82, 3.435, 3.54, 3.076, 1.03, 3.08, 6.190, 1.493};
const float motZeros[8] = {0.93, 5.712, 3.777, 3.853, 2.183, 1.556, .675, 2.679}; // RML Ellie
//const float motZeros[8] = {0.631, 4.076, 1.852, 3.414, 1.817, 5.500, 1.078, 6.252}; //RML Odie
#endif

#define timeBetweenUpdatePrints 5
uint32_t lastUpdatePrintTime = 0;

void interpData(uint32_t *x, float *y, int length);
float wrap(float ang);

// State machine representation of behavior
enum SWMode
{
	SIT = 0,
	STAND,
	FrontLegSweep,
	FrontLegSweepPrep,
	FrontLegSweepPrepReturn,
	inContact,
	processData
};

class CompliantLeg : public ReorientableBehavior
{
public:
	SWMode mode = SIT; //Current state within state-machine
	SWMode lastMode = mode;
	uint32_t tLast; //int used to store system time at various events
	minitaurVelocity motorVel;
	float lastExtension; //float for storing leg extension during the last control loop
	float exCmd;				 //The commanded leg extension
	float extDes;				 //The desired leg extension
	float angDes;				 // The desired leg angle
	float angleRate = 0.002;
	float angCommand[8];
	MinitaurShinContact contactComp = MinitaurShinContact(10,20,1.7);
	float contactX = 0;
	float contactY = 0;
	float contactLc = 0;
	int contactSamples = 0;
	int numSamples = arraySize;
	float q1Record[arraySize];
	float q2Record[arraySize];
	float q1DotRecord[arraySize];
	float q2DotRecord[arraySize];
	uint32_t tRecord[arraySize];
	float q1DotLast;
	float q2DotLast;
	int sampleCounter;


	//Maximum difference between commanded and actual leg extension
	const float maxDeltaExtCmd = 0.002;
	const float kExtAnimRate = 0.002; //Maximum rate (in m/s) of change in cmdExt

	//sig is mapped from remote; here, 3 corresponds to pushing the left stick to the right
	// which in turn forces the state machine into FH_LEAP
	void signal(uint32_t sig)
	{
		if(sig == 3)
		{
			mode = FrontLegSweep;
			tLast = S->millis;
		}
		if (sig == 2)
		{
			mode = FrontLegSweepPrep;
			tLast = S->millis;
		}
			
	}

	void begin()
	{
		motorVel.init();
		mode = FrontLegSweepPrep;			// Start behavior in STAND mode
		tLast = S->millis;		// Record the system time @ this transition
		exCmd = 0.14;					//Set extension command to be 0.14m, the current value of extension
		lastExtension = 0.14; // Record the previous loop's extension; it should be 0.14m
	}

	void update()
	{
		motorVel.updateVelocity();
		if (isReorienting())
			return;
		if (mode == SIT || mode == STAND)
			C->mode = RobotCommand_Mode_LIMB;
		else
			C->mode = RobotCommand_Mode_JOINT;
		if (mode == SIT)
		{
			for (int i = 0; i < P->limbs_count; ++i)
			{
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;
				// Splay angle for the front/rear legs (outward splay due to fore-displacement of front legs
				// and aft-displacement of rear legs)
				// The pitch angle (S->imu.euler.y) is subtracted since we want to the set the *absolute* leg angle
				// and limb[i].setPosition(ANGLE, *) will set the angle of the leg *relative* to the robot body
				angDes = (isFront(i)) ? -S->imu.euler.y - 0.1 : -S->imu.euler.y + 0.2;
				limb[i].setGain(ANGLE, 0.8, .03);
				limb[i].setPosition(ANGLE, angDes);

				limb[i].setGain(EXTENSION, 120, 3);
				// Set the leg extension to 0.14 m
				limb[i].setPosition(EXTENSION, 0.14);
			}
			lastMode = SIT;
		}
		else if (mode == STAND)
		{
			// C->behavior.pose.position.z can be commanded from the joystick (the left vertical axis by default)
			// We map this using map() to the desired leg extension, so that the joystick can be used to raise
			// and lower the standing height between 0.12 and 0.25 m
			extDes = map(C->behavior.pose.position.z, -1.0, 1.0, 0.11, 0.25);
			//If the commanded position is significantly lower than actual position,
			// and the behavior has just switched from SIT to STAND, then we smoothly
			// interpolate commanded positions between the last extension and the desired
			// extension, at the rate set by kExtAnimRate. This prevents the robot from
			// falling to quickly.
			if (S->millis - tLast < 250 && exCmd < extDes)
			{
				exCmd = exCmd + (extDes - lastExtension) * kExtAnimRate;
			}
			else
			{
				// After this initial period, or if the initial command is higher than
				// the actual initial position, we check to makes sure that the commanded
				// position is within maxDeltaExtCmd. If it is not, simply add or subtract
				// the max delta value until the difference is less than that. This prevents
				// from changing the extension faster than maxDeltaExtCmd*CONTROL_RATE m/s.
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
				angDes = (isFront(i)) ? -S->imu.euler.y - 0.01 : -S->imu.euler.y + 0.02;
				// Stiffen the angle gain linearly as a function of the extension
				// This way, more torque is provided as the moment arm becomes longer.
				limb[i].setGain(ANGLE, 0.8 + 0.2 * ((extDes - 0.12) / 0.13), 0.03);
				limb[i].setPosition(ANGLE, angDes);

				limb[i].setGain(EXTENSION, 120, 4);
				// The smoothly animated leg extension
				limb[i].setPosition(EXTENSION, exCmd);
			}
			lastMode = STAND;
		}
		else if (mode == FrontLegSweepPrep)
		{
			if (lastMode != FrontLegSweepPrep)
			{
				for (int i = 0; i < P->joints_count; ++i)
				{
					joint[i].setOpenLoop(0);
					angCommand[i] = joint[i].getPosition();
				}
			}
			float tempComm = 2.5+PI;
			if (abs(tempComm-angCommand[6]) > angleRate*(S->millis-tLast))
			{
				if (tempComm > angCommand[6])
					angCommand[6] = angCommand[6] + angleRate*(S->millis-tLast);
				else
					angCommand[6] = angCommand[6] - angleRate*(S->millis-tLast);
			}
			else
				angCommand[6] = tempComm;
			tempComm = 3.1-PI;
			if (abs(tempComm-angCommand[7]) > angleRate*(S->millis-tLast))
			{
				if (tempComm > angCommand[7])
					angCommand[7] = angCommand[7] + angleRate*(S->millis-tLast);
				else
					angCommand[7] = angCommand[7] - angleRate*(S->millis-tLast);
			}
			else
				angCommand[7] = tempComm;
			
			joint[6].setGain(1,0.01);
			joint[6].setPosition(wrap(angCommand[6]));
			joint[7].setGain(1,0.01);
			joint[7].setPosition(wrap(angCommand[7]));
			
			tLast = S->millis;
			lastMode = FrontLegSweepPrep;
		}
		else if (mode == FrontLegSweepPrepReturn)
		{
			if (lastMode != FrontLegSweepPrepReturn)
			{
				for (int i = 0; i < P->joints_count; ++i)
				{
					joint[i].setOpenLoop(0);
					angCommand[i] = joint[i].getPosition();
				}
			}
			float tempComm = 2.5-PI;
			if (abs(tempComm-angCommand[6]) > angleRate*(S->millis-tLast))
			{
				if (tempComm > angCommand[6])
					angCommand[6] = angCommand[6] + angleRate*(S->millis-tLast);
				else
					angCommand[6] = angCommand[6] - angleRate*(S->millis-tLast);
			}
			else
				angCommand[6] = tempComm;
			tempComm = 3.1-PI;
			if (abs(tempComm-angCommand[7]) > angleRate*(S->millis-tLast))
			{
				if (tempComm > angCommand[7])
					angCommand[7] = angCommand[7] + angleRate*(S->millis-tLast);
				else
					angCommand[7] = angCommand[7] - angleRate*(S->millis-tLast);
			}
			else
				angCommand[7] = tempComm;
			
			joint[6].setGain(1,0.01);
			joint[6].setPosition(wrap(angCommand[6]));
			joint[7].setGain(1,0.01);
			joint[7].setPosition(wrap(angCommand[7]));
			
			tLast = S->millis;
			lastMode = FrontLegSweepPrepReturn;
		}
		else if (mode == FrontLegSweep)
		{
			float tempComm = 1.5-PI;
			angleRate = 0.0005;
			if (abs(tempComm-angCommand[7]) > angleRate*(S->millis-tLast))
			{
				if (tempComm > angCommand[7])
					angCommand[7] = angCommand[7] + angleRate*(S->millis-tLast);
				else
					angCommand[7] = angCommand[7] - angleRate*(S->millis-tLast);
			}
			else
				angCommand[7] = tempComm;
			joint[6].setGain(0.1,0);
			joint[6].setPosition(wrap(angCommand[6]));
			joint[7].setGain(0.6,0);
			joint[7].setPosition(wrap(angCommand[7]));

			tLast =  S->millis;
			lastMode = FrontLegSweep;
			if (C->behavior.twist.angular.z < -1.1)
				mode = inContact;
		}
		else if (mode == inContact)
		{
			float tempComm = 1.5-PI;
			angleRate = 0.0005;
			if (abs(tempComm-angCommand[7]) > angleRate*(S->millis-tLast))
			{
				if (tempComm > angCommand[7])
					angCommand[7] = angCommand[7] + angleRate*(S->millis-tLast);
				else
					angCommand[7] = angCommand[7] - angleRate*(S->millis-tLast);
			}
			else
				angCommand[7] = tempComm;
			joint[6].setGain(0.1,0);
			joint[6].setPosition(wrap(angCommand[6]));
			joint[7].setGain(0.6,0);
			joint[7].setPosition(wrap(angCommand[7]));
			tLast = S->millis;
			if (lastMode != inContact)
			{
				contactX = 0;
				contactY = 0;
				contactLc = 0;
				contactSamples = 0;
				sampleCounter = 0;
			}
			q1Record[sampleCounter] = joint[7].getPosition();
			q2Record[sampleCounter] = joint[6].getPosition();
			q1DotRecord[sampleCounter] = motorVel.filteredVel[7];
			q2DotRecord[sampleCounter] = motorVel.filteredVel[6];
			tRecord[sampleCounter] = clockTimeUS;
			sampleCounter++;
			lastMode = inContact;
			if (sampleCounter == numSamples)
				mode = processData;
		}
		else if (mode == processData)
		{
			if (lastMode != processData)
			{
				sampleCounter = 0;
			}
			lastMode = processData;
			int stepSize = 50000;
			if ((S->millis - lastUpdatePrintTime) > timeBetweenUpdatePrints)
			{
				lastUpdatePrintTime = S->millis;
				float q1 = q1Record[sampleCounter];
				float q2 = q2Record[sampleCounter];
				float q1Dot = q1DotRecord[sampleCounter];
				float q2Dot = q2DotRecord[sampleCounter];
				contactLc = contactComp.findLc(q1,q2,q1Dot,q2Dot);
				float compX,compY;
				contactComp.LcToXY(q1,q2, contactLc,compX, compY);
				printf("%lu,%f,%f,%f,%f,%f,%f,%f;\n",tRecord[sampleCounter],q1Record[sampleCounter],q2Record[sampleCounter],q1Dot,q2Dot,contactLc,compX,compY);
				sampleCounter++;
				/*int forwardIndex = 1;
				while (tRecord[sampleCounter+forwardIndex] < (tRecord[sampleCounter]+stepSize))
				{
					forwardIndex++;
					if ((sampleCounter + forwardIndex)==numSamples)
						break;
				}
				if ((sampleCounter+forwardIndex)>=numSamples)
					sampleCounter = numSamples;
				else
				{
					float q1 = q1Record[sampleCounter];
					float q2 = q2Record[sampleCounter];
					float q1Dot = (q1Record[sampleCounter + forwardIndex]-q1)/((tRecord[sampleCounter+forwardIndex]-tRecord[sampleCounter])*0.000001);
					float q2Dot = (q2Record[sampleCounter + forwardIndex]-q2)/((tRecord[sampleCounter+forwardIndex]-tRecord[sampleCounter])*0.000001);
					float weight = 0.99;
					if (sampleCounter != 0)
					{
						q1Dot = weight*q1DotLast + (1-weight) * q1Dot;
						q2Dot = weight*q2DotLast + (1-weight) * q2Dot;
					}
					q1DotLast = q1Dot;
					q2DotLast = q2Dot;
					contactLc = contactComp.findLc(q1,q2,q1Dot,q2Dot);
					float compX,compY;
					contactComp.LcToXY(q1,q2, contactLc,compX, compY);
					printf("%lu,%f,%f,%f,%f,%f,%f,%f;\n",tRecord[sampleCounter],q1Record[sampleCounter],q2Record[sampleCounter],q1Dot,q2Dot,contactLc,compX,compY);
					sampleCounter++;
				}*/

			}
			if (sampleCounter == numSamples)
			{
				mode = FrontLegSweepPrepReturn;
				printf("\n\n\n");
				motorVel.dumpData();
			}
		}
	}

	bool running()
	{
		return !(mode == SIT);
	}
	void end()
	{
		mode = SIT;
	}
};

CompliantLeg *compLegPointer;

void debug()
{
}

int main(int argc, char *argv[])
{
#if defined(ROBOT_MINITAUR)
	init(RobotParams_Type_MINITAUR, argc, argv);
	for (int i = 0; i < P->joints_count; ++i)
		P->joints[i].zero = motZeros[i]; //Add motor zeros from array at beginning of file
#elif defined(ROBOT_MINITAUR_E)
	init(RobotParams_Type_MINITAUR_E, argc, argv);
	JoyType joyType = JoyType_FRSKY_XSR;
	ioctl(JOYSTICK_FILENO, IOCTL_CMD_JOYSTICK_SET_TYPE, &joyType);
#else
#error "Define robot type in preprocessor"
#endif
	setDebugRate(1);

	// Declare instance of our behavior
	CompliantLeg compLeg;
	compLegPointer = &compLeg;

	// Add our behavior to the behavior vector (Walk and Bound are already there)
	behaviors.push_back(&compLeg);

	// Change serial port baud speed. 115200, 8N1 is the default already, but this demonstrates how to change to other settings
	SerialPortConfig cfg;
	cfg.baud = 115200;
	cfg.mode = SERIAL_8N1;
	ioctl(STDOUT_FILENO, IOCTL_CMD_SERIAL_PORT_CFG, &cfg);

	return begin();
}

void interpData(uint32_t *x, float *y, int length)
{
	int i = 0;
	while (i < (length-1))
	{
		int j = 1;
		while (y[i+j] == y[i])
		{
			j++;
			if (!(i+j < length))
			{
				j = length-1-i;
				break;
			}
		}
		for (int k = 1; k < j;k++)
		{
			y[i+k] = y[i] + (double)(x[i+k]-x[i]) * (y[i+j]-y[i])/(double)(x[i+j]-x[i]);
		}
		i = i+j;
	}
}

float wrap(float ang)
{
	float post = ang - floor(ang/(2*PI))*2*PI;
	if (post > PI)
		post = post - 2*PI;
	return post;
}