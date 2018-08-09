/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Tom Jacobs <tom.jacobs@ghostrobotics.io>, Avik De <avik@ghostrobotics.io>, and Turner Topping <turner@ghostrobotics.io>
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

// Seconds passed
int timer = 0;

// Previous time in milliseconds
uint32_t oldMillis = S->millis;

class TimedWalk : public Peripheral
{
public:

	void begin()
	{
		// Command by behavior
		C->mode = RobotCommand_Mode_BEHAVIOR;

		// Start behavior
		C->behavior.mode = BehaviorMode_RUN;

		// Stand still
		C->behavior.twist.linear.x = 0.0;

		// Save time
		oldMillis = S->millis;
	}

	void update()
	{
		// Increase timer
		if(S->millis > oldMillis + 1000) 
		{
			timer++;
			oldMillis = S->millis;
		}

		// Choose behavior speed
		if(timer < 1){
			// Stand still
			C->behavior.twist.linear.x = 0.0;
		}
		else if(timer >= 1 && timer < 10)
		{
			// Walk forward slowly
			C->behavior.twist.linear.x = 0.2;
		}
		else if(timer >= 10) {
			// Reset timer
			timer = 0;
		}
	}
};

void debug()
{
	// double t = S->millis;
	// if(t > 10000)  // 10 seconds
	// {
	// 	std::cout << end << "\n";
	// 	return;
	// }

  // printf("%u %.2f %.2f %.2f %.2f\n",
	// 	S->millis, joint[0].getTorqueEst(), joint[0].getOpenLoop(),
	// 	S->batt.voltage, joint[0].getVelocity());
							//  
  // printf("%u %.2f %.2f\n",
  // printf("%u %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",
	// 	S->millis, limb[0].getForce(EXTENSION), limb[0].getForce(ANGLE),
	// 						 limb[1].getForce(EXTENSION), limb[1].getForce(ANGLE),
	// 						 limb[2].getForce(EXTENSION), limb[2].getForce(ANGLE),
	// 						 limb[3].getForce(EXTENSION), limb[3].getForce(ANGLE));
}

int main(int argc, char *argv[])
{
	init(RobotParams_Type_MINITAUR, argc, argv);
	for (int i = 0; i < P->joints_count; ++i)
		P->joints[i].zero = motZeros[i];

	// Disable joystick input
	JoyType joyType = JoyType_NONE;
	ioctl(JOYSTICK_FILENO, IOCTL_CMD_JOYSTICK_SET_TYPE, &joyType);

  setDebugRate(20);

	// Create controller peripheral
	TimedWalk timedWalk;
	timedWalk.begin();

	// Add it
	addPeripheral(&timedWalk);

	// Remove bound behavior from Minitaur (first element of behaviors vector), so we're left with only walk behavior
	behaviors.erase(behaviors.begin());

	return begin();
}
