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
enum FHMode
{
	FH_SIT = 0,
	FH_STAND,
	FH_LEAP,
	FH_LAND
};

/**
 * See "Getting started with the FirstHop behavior" in the documentation for a walk-through
 * guide.
 */
class FirstHop : public ReorientableBehavior
{
public:
	FHMode mode = FH_SIT; //Current state within state-machine

	uint32_t tLast; //int used to store system time at various events

	float lastExtension; //float for storing leg extension during the last control loop
	float exCmd;				 //The commanded leg extension
	float extDes;				 //The desired leg extension
	float angDes;				 // The desired leg angle

	bool unitUpdated;

	//Maximum difference between commanded and actual leg extension
	const float maxDeltaExtCmd = 0.002;
	const float kExtAnimRate = 0.002; //Maximum rate (in m/s) of change in cmdExt

	//sig is mapped from remote; here, 3 corresponds to pushing the left stick to the right
	// which in turn forces the state machine into FH_LEAP
	void signal(uint32_t sig)
	{
		if(sig > 1)
		{
			mode = FH_LEAP;
			tLast = S->millis;
		}
			
	}

	void begin()
	{
		mode = FH_STAND;			// Start behavior in STAND mode
		tLast = S->millis;		// Record the system time @ this transition
		exCmd = 0.14;					//Set extension command to be 0.14m, the current value of extension
		lastExtension = 0.14; // Record the previous loop's extension; it should be 0.14m
	}

	void update()
	{
		if (isReorienting())
			return;
		C->mode = RobotCommand_Mode_LIMB;
		if (mode == FH_SIT)
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
		}
		else if (mode == FH_STAND)
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
		}
		else if (mode == FH_LEAP)
		{
			for (int i = 0; i < P->limbs_count; ++i)
			{
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
				// Use setOpenLoop to exert the highest possible vertical force
				limb[i].setOpenLoop(EXTENSION, 2);  
				limb[i].setGain(ANGLE, 1.0, 0.03);
				limb[i].setPosition(ANGLE, -S->imu.euler.y);
				// After the mean leg angle passes 2.7 radians (note that we have changed the leg kinematics
				// to LimbParams_Type_SYMM5BAR_EXT_RAD) for this case, switch into a different mode (LAND)
				if (limb[i].getPosition(EXTENSION) > 2.7)
				{
					mode = FH_LAND;
					tLast = S->millis;
					unitUpdated = false;
				}
			}


			// C->mode = RobotCommand_Mode_JOINT;
			// for (int i = 0; i < P->joints_count; ++i)
			// {
			// 	// Use setOpenLoop to exert the highest possible vertical force
			// 	joint[i].setOpenLoop(1);  

			// }

			// for (int i = 0; i < P->limbs_count; ++i)
			// {
			// 	P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
			// 	// After the mean leg angle passes 2.7 radians (note that we have changed the leg kinematics
			// 	// to LimbParams_Type_SYMM5BAR_EXT_RAD) for this case, switch into a different mode (LAND)
			// 	if (limb[i].getPosition(EXTENSION) > 3 || (S->millis - tLast >= 500))
			// 	{
			// 		mode = FH_LAND;
			// 		tLast = S->millis;
			// 		unitUpdated = false;
			// 	}
			// }
			
		}
		else if (mode == FH_LAND)
		{

			for (int i = 0; i < P->limbs_count; ++i)
			{

				// This updates the parameters struct to switch back into meters as its units.
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;
				// Sets the commanded length for landing to 0.25 meters
				exCmd = 0.25;
				// Sets the desired leg angle to be facing downward plus a leg splay in the front
				// and back.
				angDes = (isFront(i)) ? -S->imu.euler.y - 0.1 : -S->imu.euler.y + 0.2;

				limb[i].setGain(ANGLE, 1.2, 0.03);
				limb[i].setPosition(ANGLE, angDes);

				limb[i].setGain(EXTENSION, 150, 5);
				limb[i].setPosition(EXTENSION, exCmd);

				// Use Limb::getForce for touchdown detection, and set a 20 millisecond
				// grace period so that the legs can settle to their landing extension,
				// without their inertia triggering a false positive.

				if (limb[i].getForce(EXTENSION) > 40 && S->millis - tLast > 20)
				{
					mode = FH_STAND;
					tLast = S->millis;
					exCmd = 0.25;
					lastExtension = 0.25;
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

class Dig : public ReorientableBehavior {
public:
  // From base class
  float standAng = 0,standExt = 1.57; 
  float kExt = 2.2;
  float kAng = 2.2;
  float tc = 10;
  int tReady = 1000/tc;
  int tLower = 500/tc;
  int tSweep = 2000/tc;
  int tRaise = tLower;
  int tReturn = tReady;
  float sweep = -0.5;
  float lift = -1.6;

  void begin()
  {};

  void update(){

  	if (isReorienting())
			return;

  	// for (int i = 0; i < P->limbs_count; ++i){
	  // 	P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
	  // }
	  	  
	  // for (int i = 0; i < P->limbs_count; ++i){
	  //   limb[i].setGain(ANGLE, kAng);
	  //   limb[i].setGain(EXTENSION, kExt);
	  //   limb[i].setPosition(ANGLE, isFront(i) ? 0.3 : -0.3);
	  // 	limb[i].setPosition(EXTENSION, standExt);
	  // }


	  
  	  for (int i = 0; i < P->limbs_count; ++i){
	  	P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
	  }
	  
	  
	  int time = S->millis % (tLower+tReady+tSweep+tRaise+tReturn);
	  
	  for (int i = 0; i < P->limbs_count; ++i){
	    limb[i].setGain(ANGLE, kAng);
	    limb[i].setGain(EXTENSION, kExt);
	  }
	  limb[2].setGain(EXTENSION, kExt/3);

	  limb[0].setPosition(ANGLE, standAng);
	  limb[0].setPosition(EXTENSION, standExt);
	  limb[3].setPosition(ANGLE, standAng);
	  limb[3].setPosition(EXTENSION, standExt);

	  limb[1].setPosition(EXTENSION, standExt/4);
	  
	  standExt = 2;  
	  
	  if (time<tReady){
	      limb[2].setPosition(ANGLE, standAng + (float)time*(sweep)/(float)(tReady-0));
	      limb[2].setPosition(EXTENSION, standExt + lift);
	  }
	  else if(time>=(tReady) && time<(tReady+tLower)){
	      limb[2].setPosition(ANGLE, standAng + sweep);
	      limb[2].setPosition(EXTENSION, (standExt+lift) + (float)(time-tReady)*(-lift)/(float)(tLower));
	  }
	  else if(time>=(tReady+tLower) && time<(tReady+tLower+tSweep)){
	      limb[2].setPosition(ANGLE, (standAng+sweep) + (float)(time-(tReady+tLower))*(-2*sweep)/(float)(tSweep));
	      limb[2].setPosition(EXTENSION, standExt);
	  }
	  else if(time>=(tReady+tLower+tSweep) && time<(tReady+tLower+tSweep+tRaise)){
	      limb[2].setPosition(ANGLE, standAng - sweep);
	      limb[2].setPosition(EXTENSION, (standExt) + (float)(time-(tReady+tLower+tSweep))*(lift)/(float)(tRaise));
	  }
	  else{
	      limb[2].setPosition(ANGLE, (standAng - sweep) + (float)(time-(tReady+tLower+tSweep+tRaise))*(sweep)/(float)(tReturn));
	      limb[2].setPosition(EXTENSION, standExt + lift); 
	  }

  };
  bool running() {
    return false;
  }
  void end() {
  }
  void signal() {
  }
};

void debug()
{

	for (int i = 0; i < P->joints_count; ++i)
		{
			// Use setOpenLoop to exert the highest possible vertical force
			printf("Motor %d command: %4.3f \n",i, joint[i].getOpenLoop());  
		}
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
	FirstHop firstHop;
	Dig dig;

	// Add our behavior to the behavior vector (Walk and Bound are already there)
	behaviors.push_back(&firstHop);
	behaviors.push_back(&dig);

	// Change serial port baud speed. 115200, 8N1 is the default already, but this demonstrates how to change to other settings
	SerialPortConfig cfg;
	cfg.baud = 115200;
	cfg.mode = SERIAL_8N1;
	ioctl(STDOUT_FILENO, IOCTL_CMD_SERIAL_PORT_CFG, &cfg);

	return begin();
}
