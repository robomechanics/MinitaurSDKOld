/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io> and Tom Jacobs <tom.jacobs@ghostrobotics.io>
 */
#include <stdio.h>
#include <SDK.h>
#include <Motor.h>

enum spinMode
{
	KILL,
	RUN
};

float avg(float *myArray, int len){
	float sum = 0;
	float result;
	for (int i = 0; i < len; i++){
	sum += myArray[i];
	}
	result = sum/float(len);
	return result;
}

float filteredVelocity; // global variable for debug()

class Test : public Behavior
{
public:
	spinMode mode = RUN;
	float df = 0.6;
	const static int bufferSize = 1000;
	float velocityBuffer[bufferSize];
	int counter;
	bool filledBuffer = false;
	float curVel;

	void signal(uint32_t sig)
	{
		if(sig == 3){
			mode = KILL;
		}
		if (sig == 2){
			mode = RUN;
		}
			
	}
	void begin()
	{
		counter = 0;
		mode = KILL;
	}

	void update()
	{
		// Control robot by joints
		C->mode = RobotCommand_Mode_JOINT;

		for (int i=0; i<8; ++i){
			C->joints[i].mode = JointMode_OFF; // turn off leg motors
		}
		if (mode == RUN){
			// Spin tail motor
			joint[8].setOpenLoop(df);
		} else if (mode == KILL){
			joint[8].setOpenLoop(0);
		}

		// Read motor speed
		curVel = joint[8].getVelocity();
		if (counter == bufferSize - 1){
			filledBuffer = true;
		}
		velocityBuffer[counter] = curVel;
		counter = (counter+1) % bufferSize;

		// Calculate motor velocity
		if (filledBuffer){
			filteredVelocity = avg(velocityBuffer, bufferSize);
		} else{
			filteredVelocity = avg(velocityBuffer, counter);
		}
	}

	void end()
	{

	}
};

void debug()
{
	float curVel = filteredVelocity;
	float df = joint[8].getOpenLoop();
	float pos = joint[8].getPosition();
	printf("velocity = %f,\tduty factor = %f,\tMotor position = %f\n", curVel, df, pos);
}

int main(int argc, char *argv[])
{
	// Loads some default settings including motor parameters
#if defined(ROBOT_MINITAUR)
	init(RobotParams_Type_MINITAUR, argc, argv);
	// Set the joint type; see JointParams
	P->joints[8].type = JointParams_Type_GRBL;

	// Set the *physical* address (e.g. etherCAT ID, PWM port, dynamixel ID, etc.)
	P->joints[8].address = 8;

	P->joints[8].gearRatio = 1.0;
#elif defined(ROBOT_MINITAUR_E)
	init(RobotParams_Type_MINITAUR_E, argc, argv);
#else
#error "Define robot type in preprocessor"
#endif

	// Configure joints
	#define NUM_MOTORS 9
	const float zeros[NUM_MOTORS] = {2.82, 3.435, 3.54, 3.076, 1.03, 3.08, 6.190, 1.493, 0};
	const float directions[NUM_MOTORS] = {1, 1, 1, 1, 1, 1, 1, 1, 1};
	P->joints_count = S->joints_count = C->joints_count = NUM_MOTORS;
	for (int i = 0; i < P->joints_count; i++)
	{
		// Set zeros and directions
		P->joints[i].zero = zeros[i];
		P->joints[i].direction = directions[i];
	}

	// No limbs, only 1DOF joints
	P->limbs_count = 0; 

	// Remove default behaviors from behaviors vector, create, add, and start ours
	Test test;
	safetyShutoffEnable(false);
	behaviors.clear();
	behaviors.push_back(&test);
	test.begin();
	setDebugRate(100);
	softStartEnable(false);

	SerialPortConfig cfg;
	cfg.baud = 115200;
	cfg.mode = SERIAL_8N1;
	ioctl(STDOUT_FILENO, IOCTL_CMD_SERIAL_PORT_CFG, &cfg);


	return begin();
}
