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

#if defined(ROBOT_MINITAUR)
// Subject to change for individual robots
// const float motZeros[8] = {2.82, 3.435, 3.54, 3.076, 1.03, 3.08, 6.190, 1.493};
// const float motZeros[8] = {2.570, 2.036, 3.777, 3.853, 2.183, 1.556, .675, 2.679}; // RML Ellie
const float motZeros[8] = {0.93, 5.712, 3.777, 3.853, 2.183, 1.556, .675, 2.679}; // RML Ellie
// const float motZeros[8] = {0.631, 4.076, 1.852, 3.414, 1.817, 5.500, 1.078, 6.252}; //RML Odie
#endif

// State machine representation of behavior
enum PoseBehaviorMode
{
	PoseBehavior_LOCK = 0,
	PoseBehavior_MOVE,
};

/**
 * See "Getting started with the PoseBehavior behavior" in the documentation for a walk-through
 * guide.
 */
class PoseBehavior : public ReorientableBehavior
{
public:
	PoseBehaviorMode mode = PoseBehavior_LOCK; //Current state within state-machine

	uint32_t tLast; //int used to store system time at various events

	float lastExtension; //float for storing leg extension during the last control loop
	float exCmd;				 //The commanded leg extension
	float extDes;				 //The desired leg extension
	float angDes;				 // The desired leg angle

	float jointPos[8] = {1.57,1.57,1.57,1.57,1.57,1.57,1.57,1.57};

	bool unitUpdated;

	//Maximum difference between commanded and actual leg extension
	const float maxDeltaExtCmd = 0.002;
	const float kExtAnimRate = 0.002; //Maximum rate (in m/s) of change in cmdExt

	//sig is mapped from remote; here, 3 corresponds to pushing the left stick to the right
	// which in turn forces the state machine into PoseBehavior_LEAP
	void signal(uint32_t sig)
	{
		// if(sig > 1)
		// {
		// 	mode = PoseBehavior_LEAP;
		// 	tLast = S->millis;
		// }
			
	}

	void begin()
	{
		mode = PoseBehavior_MOVE;			// Start behavior in STAND mode
		tLast = S->millis;		// Record the system time @ this transition
		exCmd = 0.14;					//Set extension command to be 0.14m, the current value of extension
		lastExtension = 0.14; // Record the previous loop's extension; it should be 0.14m
	}

	void update()
	{
		if (isReorienting())
			return;
		C->mode = RobotCommand_Mode_JOINT;
		if (mode == PoseBehavior_LOCK)
		{
			for (int i = 0; i < P->joints_count; ++i)
			{
				joint[i].setGain(0.8, 0.005);
				joint[i].setPosition(jointPos[i]);
			}
		}
		else if (mode == PoseBehavior_MOVE)
		{
			for (int i = 0; i < P->joints_count; ++i)
			{
				joint[i].setGain(0.0, 0.020);
				joint[i].setPosition(jointPos[i]);
			}
		}
	}

	bool running()
	{
		return !(mode == PoseBehavior_LOCK);
	}
	void end()
	{
		mode = PoseBehavior_LOCK;
		for (int i = 0; i < P->joints_count; ++i)
		{
			jointPos[i] = joint[i].getPosition();
		}		
	}
};

// Declare instance of our behavior
PoseBehavior poseBehavior;

void debug()
{

	for (int i = 0; i < P->joints_count; ++i)
	{
		// Use setOpenLoop to exert the highest possible vertical force
		// printf("Motor %d command: %4.3f \n",i, joint[i].getOpenLoop());
		printf("Motor %d: %6.3f  ",i, poseBehavior.jointPos[i]);   
	}
	printf("\n");
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

	setDebugRate(10);

	// Add our behavior to the behavior vector (Walk and Bound are already there)
	behaviors.push_back(&poseBehavior);

	// Change serial port baud speed. 115200, 8N1 is the default already, but this demonstrates how to change to other settings
	SerialPortConfig cfg;
	cfg.baud = 115200;
	cfg.mode = SERIAL_8N1;
	ioctl(STDOUT_FILENO, IOCTL_CMD_SERIAL_PORT_CFG, &cfg);

	return begin();
}
