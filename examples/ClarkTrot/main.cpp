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
const float motZeros[8] = {5.52, 5.34, 5.95, 3.81, 3.84, 4.07, 5.71, 5.38}; // RML Ellie
#endif

//#define _USE_MATH_DEFINES

// State machine representation of behavior
enum CTMode
{
	CT_SIT = 0,
	CT_STAND,
	CT_WALK,
	CT_LAND
};

/**
 * See "Getting started with the FirstHop behavior" in the documentation for a walk-through
 * guide.
 */
class ClarkTrot : public ReorientableBehavior
{
public:
	CTMode mode = CT_SIT; //Current state within state-machine

	uint32_t tLast; //int used to store system time at various events

	float lastExtension; //float for storing leg extension during the last control loop
	float exCmd;				 //The commanded leg extension
	float extDes;				 //The desired leg extension
	float angDes;				 // The desired leg angle

	float angCmd; //angle command
	float yawCmd; //steering command
	float yawDes;
	float yawMax = 0.09;
	float yawOffset;

	bool unitUpdated;

	//Maximum difference between commanded and actual leg extension
	const float maxDeltaExtCmd = 0.002;
	const float kExtAnimRate = 0.002; //Maximum rate (in m/s) of change in cmdExt


	//For walk code
	float freq = 2.0; //4.750; //4750; //Frequency of gait (Hz)
	uint32_t df = 50; //Duty Factor (%)
	float strokeLen = 0.12;//0.16; //Stroke Length (m)
	float beta = 40; //60; //Approach Angle (deg)
	float Kp = 180; //1.70; //Proportional Gain 120, 4
	float Kd = 1.8; //0.018; //Derivative Gain

	float extRetract = 0.065; //Extension Retraction Height (m)
	float gcl = 0.17; //min ground clearance (m)

	float angNom;//= 10.0/180*M_PI; // nominal foot angle
	float angOffset = 10.0/180*M_PI;

	uint32_t tStart; //gait start time (ms)
	uint32_t cTime; //cycle time (ms)

	float temp1;
	float temp2;
	float temp3;
	float temp4;


	//sig is mapped from remote; here, 3 corresponds to pushing the left stick to the right
	// which in turn forces the state machine into FH_LEAP
	void signal(uint32_t sig)
	{
		if(sig > 1)
		{
			//mode = CT_WALK;
			//tStart = S->millis;
		}
			
	}

	void begin()
	{
		mode = CT_SIT;			// Start behavior in STAND mode
		tStart = S->millis;		// Record the system time @ this transition
		exCmd = 0.14;					//Set extension command to be 0.14m, the current value of extension
		lastExtension = 0.14; // Record the previous loop's extension; it should be 0.14m
	}

	void update()
	{	
		if(C->behavior.twist.angular.z > 0.5) {
			mode = CT_WALK;
		}
		else {
			mode = CT_SIT;
		}
		C->mode = RobotCommand_Mode_JOINT;
		if (isReorienting())
			return;
		C->mode = RobotCommand_Mode_LIMB;
		if (mode == CT_SIT)
		{
			for (int i = 0; i < P->limbs_count; ++i)
			{
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;
				// Splay angle for the front/rear legs (outward splay due to fore-displacement of front legs
				// and aft-displacement of rear legs)
				// The pitch angle (S->imu.euler.y) is subtracted since we want to the set the *absolute* leg angle
				// and limb[i].setPosition(ANGLE, *) will set the angle of the leg *relative* to the robot body
				angDes = (isFront(i)) ? -S->imu.euler.y + 5.0/180*M_PI : -S->imu.euler.y + 5.0/180*M_PI;
				limb[i].setGain(ANGLE, 0.8, .03);
				limb[i].setPosition(ANGLE, angDes);

				limb[i].setGain(EXTENSION, 120, 3);
				// Set the leg extension to 0.14 m
				limb[i].setPosition(EXTENSION, 0.14);
			}
		}
		else if (mode == CT_STAND)
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
		else if (mode == CT_WALK)
		{

			float stanceExt = sqrt(pow(strokeLen/2,2)+pow(gcl,2));
			float stanceAng = atan((strokeLen/2)/gcl);

			float h4 = tan(beta*M_PI/180)*(strokeLen/3);
			float phi = atan((strokeLen*5/6)/(gcl-h4));
			float ext4 = sqrt(pow(strokeLen*5/6,2)+pow(gcl-h4,2));

			float matchFrac = (phi-stanceAng)/(stanceAng/(df/2)); //(df/stanceAng)*(phi-stanceAng);
			float resetFrac = 100 - df - matchFrac;
			float gclFrac = resetFrac/(phi+stanceAng)*stanceAng;

			cTime = (S->millis - tStart); //cycle time

			yawDes = 0.0; // map(C->behavior.twist.angular.z, -1, 1, -yawMax, yawMax);

			for (int i = 0; i < P->limbs_count; ++i)
			{
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;

				//Diagonal Pairs
				uint32_t legPhase = (i==0 || i==3) ? 0 : 1;
				//Left/Right Pairs
				yawCmd = (i==0 || i==1) ?  -yawDes: yawDes;
				//Front/Back Pairs
				angNom = isFront(i) ?  10.0/180*M_PI: 10.0/180*M_PI;


				float frac = (fmod(cTime + (legPhase*(1/freq)/2*1000), 1000/freq))*freq*0.1;

				if(frac <= df) {
					//stance phase (pt 1 to 2)
					angCmd = map(frac, 0, df, angNom+yawCmd-stanceAng, angNom+yawCmd+stanceAng);
					exCmd = stanceExt;
				}
			    else if(frac > df) {
			    	//flight phase
			    	if (frac < df+resetFrac){
			    		// Flight Leg Reset
			    		angCmd = map(frac, df, df+resetFrac, angNom+stanceAng, angNom-phi);

			    		if (frac < df+gclFrac) {
			    			// Ground Clearance (point 2 to 3)
			    			exCmd = map(frac, df, df+gclFrac, stanceExt, gcl-extRetract);

			    		}
			    		else if (frac > df+gclFrac) {
			    			//Approach Angle (pt 3 to 4)
			    			exCmd = map(frac, df+gclFrac, df+resetFrac, gcl-extRetract, ext4);
			    		}

			    	}
			    	else if(frac > df+resetFrac) {
			    		// Ground Speed Matching (pt 4 to 1)
			    		angCmd = map(frac, df+resetFrac, df+resetFrac+matchFrac, angNom-phi, angNom-stanceAng);
			    		exCmd = map(frac, df+resetFrac, df+resetFrac+matchFrac, ext4, stanceExt);
			    	}
			    }

			    limb[i].setGain(ANGLE, 2.2, .03);
				limb[i].setGain(EXTENSION, Kp, Kd);

			    limb[i].setPosition(ANGLE,angCmd);
			    limb[i].setPosition(EXTENSION,exCmd);
			}
		}
	}

	bool running()
	{
		return !(mode == CT_SIT);
	}
	void end()
	{
		mode = CT_SIT;
	}
};

ClarkTrot cTrot;

void debug()
{

    //printf("time : %lu\n", cTrot.cTime);
  	//printf("\n");
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

	setDebugRate(100);

	// Declare instance of our behavior
	//ClarkTrot cTrot;

	//behaviors.clear();
	// Add our behavior to the behavior vector (Walk and Bound are already there)
  	behaviors.push_back(&cTrot);

	// Change serial port baud speed. 115200, 8N1 is the default already, but this demonstrates how to change to other settings
	SerialPortConfig cfg;
	cfg.baud = 115200;
	cfg.mode = SERIAL_8N1;
	ioctl(STDOUT_FILENO, IOCTL_CMD_SERIAL_PORT_CFG, &cfg);

	return begin();
}
