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
const float motZeros[8] = {2.570, 2.036, 3.777, 3.853, 2.183, 1.556, .675, 2.679}; // RML Ellie
// const float motZeros[8] = {0.631, 4.076, 1.852, 3.414, 1.817, 5.500, 1.078, 6.252}; //RML Odie
#endif


/**
 * See "Getting started with the FirstHop behavior" in the documentation for a walk-through
 * guide.
 */
class LegTest : public ReorientableBehavior
{
public:

	void signal(uint32_t sig)
	{

	}

	void begin()
	{

	}

	void update()
	{
		if (isReorienting())
			return;
		C->mode = RobotCommand_Mode_JOINT;

		for (int i = 0; i < P->joints_count; ++i)
		{
			joint[i].setGain(0.5);
			joint[i].setPosition(1.57);
		}


	}

	bool running()
	{

	}
	void end()
	{

	}
};


void debug()
{


	float command = limb[0].getPosition(EXTENSION);
	printf("Command: %4.3f \n", command);
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

	setDebugRate(50);

	// Declare instance of our behavior
	LegTest legTest;

	// Add our behavior to the behavior vector (Walk and Bound are already there)
	behaviors.clear();
	behaviors.push_back(&legTest);
	legTest.begin();
	
	// Change serial port baud speed. 115200, 8N1 is the default already, but this demonstrates how to change to other settings
	SerialPortConfig cfg;
	cfg.baud = 115200;
	cfg.mode = SERIAL_8N1;
	ioctl(STDOUT_FILENO, IOCTL_CMD_SERIAL_PORT_CFG, &cfg);

	return begin();
}
