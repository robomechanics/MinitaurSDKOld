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

class MyBehavior : public Behavior {

    void begin() {

        // Control the robot via joints
        C->mode = RobotCommand_Mode_JOINT;
     }

    void update() {

        // Make motor 0 spin slowly
        joint[0].setOpenLoop(0.5);
    }

    void end()
    {

    }
};

void debug()
{
	double t = S->millis;
	if(t > 10000)  // 10 seconds
	{
		std::cout << end << "\n";
		return;
	}

  printf("%u %.2f %.2f %.2f %.2f\n",
		S->millis, limb[0].getForce(EXTENSION),
							 limb[1].getForce(EXTENSION),
							 limb[2].getForce(EXTENSION),
							 limb[3].getForce(EXTENSION));
}

int main(int argc, char *argv[])
{
	init(RobotParams_Type_MINITAUR, argc, argv);
	for (int i = 0; i < P->joints_count; ++i)
		P->joints[i].zero = motZeros[i]; //Add motor zeros from array at beginning of file

  setDebugRate(5);

	IndWalk IndWalk;								 // Declare instance of our behavior
	behaviors.push_back(&IndWalk);

	return begin();
}
