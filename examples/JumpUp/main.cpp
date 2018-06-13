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

// Subject to change for individual robots
// const float motZeros[8] = { 2.82, 3.435, 3.54, 3.076, 1.03, 3.08, 6.190, 1.493 };
const float motZeros[8] = {2.570, 3.167, 3.777, 3.853, 2.183, 1.556, .675, 2.679}; // RML Ellie
// const float motZeros[8] = {0.631, 4.076, 1.852, 3.414, 1.817, 5.500, 1.078, 6.252}; //RML Odie


// State machine representation of behavior
enum FHMode {
	FH_SIT = 0, FH_STAND, FH_PRELEAP, FH_LEAP, FH_WAIT_FOR_UPRIGHT, FH_JUMP_UP, FH_LAND,  FH_ABSORB
};

/**
 * See "Getting started with the FirstHop behavior" in the documentation for a walk-through
 * guide.
 */
class FirstHop: public ReorientableBehavior {
public:
	FHMode mode = FH_SIT; //Current state within state-machine

	uint32_t tLast; //int used to store system time at various events

	float lastExtension; //float for storing leg extension during the last control loop
	float exCmd; //The commanded leg extension
	float extDes; //The desired leg extension
	float angDes; // The desired leg angle

	bool unitUpdated;

	//Maximum difference between commanded and actual leg extension
	const float maxDeltaExtCmd = 0.002; 
	const float kExtAnimRate = 0.002; //Maximum rate (in m/s) of change in cmdExt

	//sig is mapped from remote; here, 3 corresponds to pushing the left stick to the right
	// which in turn forces the state machine into FH_LEAP
	void signal(uint32_t sig) {
		if (sig == 3) {
			mode = FH_PRELEAP;
			tLast = S->millis;
		}
	}


	void begin() {
		mode = FH_STAND; // Start behavior in STAND mode
		tLast = S->millis; // Record the system time @ this transition
		exCmd = 0.14; //Set extension command to be 0.14m, the current value of extension
		lastExtension = 0.14; // Record the previous loop's extension; it should be 0.14m 
	}

	void update() {
		if (isReorienting() && mode != FH_WAIT_FOR_UPRIGHT && mode != FH_JUMP_UP && mode != FH_ABSORB)
			return;
		C->mode = RobotCommand_Mode_LIMB;
		if (mode == FH_SIT) {
			for (int i = 0; i < P->limbs_count; ++i) {
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;
				// Splay angle for the front/rear legs (outward splay due to fore-displacement of front legs
				// and aft-displacement of rear legs)
				// The pitch angle (S->imu.euler.y) is subtracted since we want to the set the *absolute* leg angle
				// and limb[i].setPosition(ANGLE, *) will set the angle of the leg *relative* to the robot body
				angDes = (isFront(i)) ? -S->imu.euler.y - 0.1 : -S->imu.euler.y + 0.2;
				limb[i].setGain(ANGLE, 0.8, .03);
				limb[i].setPosition(ANGLE, angDes);

				limb[i].setGain(EXTENSION, 120, 5);
				// Set the leg extension to 0.14 m
				limb[i].setPosition(EXTENSION, 0.14);

			}

		} else if (mode == FH_STAND) {
			// C->behavior.pose.position.z can be commanded from the joystick (the left vertical axis by default)
			// We map this using map() to the desired leg extension, so that the joystick can be used to raise
			// and lower the standing height between 0.12 and 0.25 m
			extDes = map(C->behavior.pose.position.z, -1.0, 1.0, 0.10, 0.25);
			//If the commanded position is significantly lower than actual position,
			// and the behavior has just switched from SIT to STAND, then we smoothly
			// interpolate commanded positions between the last extension and the desired
			// extension, at the rate set by kExtAnimRate. This prevents the robot from
			// falling to quickly.  
			if (S->millis - tLast < 250 && exCmd < extDes) {
				exCmd = exCmd + (extDes - lastExtension) * kExtAnimRate;
			} else {
				// After this initial period, or if the initial command is higher than 
				// the actual initial position, we check to makes sure that the commanded
				// position is within maxDeltaExtCmd. If it is not, simply add or subtract
				// the max delta value until the difference is less than that. This prevents
				// from changing the extension faster than maxDeltaExtCmd*CONTROL_RATE m/s. 
				if (extDes - exCmd > maxDeltaExtCmd) {
					exCmd = exCmd + maxDeltaExtCmd;
				} else if (exCmd - extDes > maxDeltaExtCmd) {
					exCmd = exCmd - maxDeltaExtCmd;
				} else {
					exCmd = extDes;
				}
			}
			for (int i = 0; i < P->limbs_count; ++i) {
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;
				// Leg splay
				angDes = (isFront(i)) ? -S->imu.euler.y - 0.1 : -S->imu.euler.y + 0.2;
				// Stiffen the angle gain linearly as a function of the extension
				// This way, more torque is provided as the moment arm becomes longer.
				limb[i].setGain(ANGLE, 1.2 + 0.2 * ((extDes - 0.12) / 0.13), 0.03);
				limb[i].setPosition(ANGLE, angDes);

				limb[i].setGain(EXTENSION, 120, 4);
				// The smoothly animated leg extension
				limb[i].setPosition(EXTENSION, exCmd);

			}

		} else if (mode == FH_PRELEAP) {


			for (int i = 0; i < P->limbs_count; ++i) {
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[i].setGain(ANGLE, 1.5, 0.03);
				limb[i].setGain(EXTENSION, 120, 4);
				angDes = (isFront(i)) ? -S->imu.euler.y - 0.1 : -S->imu.euler.y + 0.2;
				if(isFront(i)) {
					limb[i].setPosition(ANGLE, angDes);
					limb[i].setPosition(EXTENSION, 0.28);
				} else {
					limb[i].setPosition(ANGLE, angDes);
					limb[i].setPosition(EXTENSION, 0.12);
				}
				
				
				// After the mean leg angle passes 2.7 radians (note that we have changed the leg kinematics
				// to LimbParams_Type_SYMM5BAR_EXT_RAD) for this case, switch into a different mode (LAND)
				if (S->millis-tLast > 1000) {
					mode = FH_LEAP;
					tLast = S->millis;
					unitUpdated = false;
				}
			}
		} else if (mode == FH_LEAP) {

			for (int i = 0; i < P->limbs_count; ++i) {
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
				// Use setOpenLoop to exert the highest possible vertical force
				limb[i].setGain(ANGLE, 1.0, 0.03);
				angDes = (isFront(i)) ? -S->imu.euler.y - 0.1 : -S->imu.euler.y + 0.2;
				if(isFront(i)) {
					limb[i].setGain(EXTENSION, 0.4, 0.01);
					limb[i].setPosition(EXTENSION, 0.5);
					limb[i].setPosition(ANGLE, angDes);
				} else {
					limb[i].setOpenLoop(EXTENSION, 3);
					limb[i].setPosition(ANGLE, angDes);
				}
				
				
				// After the mean leg angle passes 2.7 radians (note that we have changed the leg kinematics
				// to LimbParams_Type_SYMM5BAR_EXT_RAD) for this case, switch into a different mode (LAND)
				if (limb[i].getPosition(EXTENSION) > 3 || S->millis - tLast >=500) {
					mode = FH_WAIT_FOR_UPRIGHT;
					tLast = S->millis;
					unitUpdated = false;
				}
			}
		} else if (mode == FH_WAIT_FOR_UPRIGHT) {

			for (int i = 0; i < P->limbs_count; ++i) {
				
				// This updates the parameters struct to switch back into meters as its units.
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;
				// Sets the commanded length for landing to 0.25 meters
				exCmd = (i==0 || i ==2) ? 0.11 : 0.25;
				// Sets the desired leg angle to be facing downward plus a leg splay in the front
				// and back. 
				angDes = (i==0 || i ==2) ?  -S->imu.euler.y : S->imu.euler.y;
				
				limb[i].setGain(ANGLE, 1.2, 0.03);
				limb[i].setPosition(ANGLE, angDes);

				limb[i].setGain(EXTENSION, 150, 1);
				limb[i].setPosition(EXTENSION, exCmd);

				// Use Limb::getForce for touchdown detection, and set a 20 millisecond
				// grace period so that the legs can settle to their landing extension, 
				// without their inertia triggering a false positive. 
			
				if (limb[0].getPosition(ANGLE) <= -1.4 || S->millis - tLast >=500) {
					mode = FH_JUMP_UP;
					tLast = S->millis;
					exCmd = 0.25;
					lastExtension = 0.25;

				}
			}
		} else if (mode == FH_JUMP_UP) {

			// for (int i = 0; i < P->joints_count; ++i)
			// {
			// 	// Use setOpenLoop to exert the highest possible vertical force
			// 	if ((i == 0) || (i == 1) || (i == 4) || (i == 5))
			// 	{
			// 		joint[i].setOpenLoop(1);
			// 	}
			// 	// else
			// 	// {
			// 	// 	joint[i].setGain(1.0);
			// 	// 	joint[i].setPosition((i==2 || i == 7) ? PI/4 : -3*PI/4);
			// 	// }

			// }

			for (int i = 0; i < P->limbs_count; ++i) {
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
				// Use setOpenLoop to exert the highest possible vertical force
				limb[i].setGain(ANGLE, 1.0, 0.03);
				angDes = (i==0 || i == 2) ? -S->imu.euler.y - 0.1 : -S->imu.euler.y+PI;
				if(i==1 || i ==3)
				{
					limb[i].setGain(EXTENSION, 0.4, 0.01);
					limb[i].setPosition(EXTENSION, 2.8);
					limb[i].setPosition(ANGLE, angDes);
				}
				else
				{
					limb[i].setOpenLoop(EXTENSION, 2);
					limb[i].setPosition(ANGLE, angDes);
				}
				
				
				// After the mean leg angle passes 2.7 radians (note that we have changed the leg kinematics
				// to LimbParams_Type_SYMM5BAR_EXT_RAD) for this case, switch into a different mode (LAND)
				if (limb[i].getPosition(EXTENSION) > 3  || S->millis - tLast >=250) {
					mode = FH_ABSORB;
					tLast = S->millis;
					unitUpdated = false;
				}
			}
		} else if (mode == FH_ABSORB) {

			for (int i = 0; i < P->limbs_count; ++i) {
				
				// This updates the parameters struct to switch back into meters as its units.
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;
				// Sets the commanded length for landing to 0.25 meters
				exCmd = 0.25;
				// Sets the desired leg angle to be facing downward plus a leg splay in the front
				// and back. 
				angDes = (i==0 || i == 2) ?  -S->imu.euler.y - 0.1 : S->imu.euler.y + 0.4;
				extDes = (i==0 || i == 2) ?  0.25 : 0.2;

				limb[i].setGain(ANGLE, 2, 0.03);
				limb[i].setPosition(ANGLE, angDes);

				if(i==0 || i == 2){
					limb[i].setGain(EXTENSION, 150, 1);
					limb[i].setPosition(EXTENSION, exCmd);
				} else {
					limb[i].setGain(EXTENSION, 150, 3);
					limb[i].setPosition(EXTENSION, exCmd);
				}
				

				if (S->millis-tLast > 750) {
					mode = FH_STAND;
					tLast = S->millis;
					exCmd = 0.25;
					lastExtension = 0.25;

				}
			}
		}

		printf("ADC reads %d\n", mode);
	}

	bool running() {
		return !(mode == FH_SIT);
	}
	void end() {
		mode = FH_SIT;
	}
};

FirstHop firstHop; // Declare instance of our behavior
void debug() {
	printf("isFront: %d, isRight: %d\n", firstHop.isFront(2),firstHop.isRight(2));
}

int main(int argc, char *argv[]) {
	init(RobotParams_Type_MINITAUR, argc, argv);
	for (int i = 0; i < P->joints_count; ++i)
		P->joints[i].zero = motZeros[i]; //Add motor zeros from array at beginning of file
	
	// Add our behavior to the behavior vector (Walk and Bound are already there)
	behaviors.push_back(&firstHop); 

	return begin();
}
