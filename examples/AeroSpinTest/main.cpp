/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io> and Tom Jacobs <tom.jacobs@ghostrobotics.io>
 */
#include <stdio.h>
#include <SDK.h>
#include <Motor.h>

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
	float df = 0.1;
	const int bufferSize = 100;
	float velocityBuffer[bufferSize];
	int counter;
	bool filledBuffer = false;
	float curVel;
	void begin()
	{
		counter = 0;
	}

	void update()
	{
		// Control robot by joints
		C->mode = RobotCommand_Mode_JOINT;

		for (int i=0; i<8; ++i){
			C->joints[i].mode = JointMode_OFF; // turn off leg motors
		}
		// Spin tail motor
		joint[8].setOpenLoop(df);

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
	printf("Motor velocity = %f\n", filteredVelcoity);
}

int main(int argc, char *argv[])
{
	// Loads some default settings including motor parameters
#if defined(ROBOT_MINITAUR)
	init(RobotParams_Type_MINITAUR, argc, argv);
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
	}

	// No limbs, only 1DOF joints
	P->limbs_count = 0; 

	// Remove default behaviors from behaviors vector, create, add, and start ours
	Test test;
	behaviors.clear();
	behaviors.push_back(&test);
	test.begin();

	return begin();
}
