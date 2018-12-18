/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Gavin Kenneally, Avik De, Turner Topping <gavin@ghostrobotics.io> <avik@ghostrobotics.io> <turner@ghostrobotics.io>
 */

// modified from Cameron's steady state tail behavior
#include <stdio.h>
#include <SDK.h>
#include <math.h>
#include <Smath.h>
#include <Motor.h>
#include <Behavior.h>

#include <unistd.h>

#define velocity_buffer 5
#define roll_accel_buffer 10

float theVel;
float tailVel;
bool newTest = true;
float roll_accel;


/**
 * State machine representation:
 * TS_WAIT -> stop the tail at 0 position
 * TS_SPIN -> spin the tail
 */

enum TSMode {
	TS_WAIT = 0, TS_SPIN
};
TSMode mode;
const float motZeros[9] = {2.570, 2.036, 3.777, 3.853, 2.183, 1.556, .675, 2.679, 1.000}; // RML Ellie with tail
//const float motZeros[9] = {0.631, 4.076, 1.852, 3.414, 1.817, 5.500, 1.078, 6.252, 1}; //RML Odie
float avg(float *myArray, int len){
  float sum = 0;
  float result;
  for (int i = 0; i < len; i++){
    sum += myArray[i];
  }
  result = sum/float(len);
  return result;
}

bool almostEq(float x, float y){
	float delta = 1e-5;
	return abs(x-y)<= delta;
}

void debug(){
	if(mode == TS_SPIN){
		float DF =joint[8].getOpenLoop();
		float tail_angle = limb[4].getPosition(0)+PI/2;
		float roll_vel = S->imu.angular_velocity.y;
		if(newTest){
			printf("XXXXXXXXXXX new trial, DF= ");
			printf(" XXXXXXXXXXX\n");
			
			newTest = false;
		}
		printf("DF = %f, tail position = %f, roll roll_accel = %f \n", DF, tail_angle, roll_accel);
	
	}
}

void myKinematics(const float *kinParams, const float *jointAngs, float *limbPos, float *JacRowmajor){
	JacRowmajor[0] = {1};
	limbPos[0] = jointAngs[0];
}

class TailSpin : public Behavior {
public:
	//TSMode mode; //Current state within state-machine

	float posDes; //Desired position
	float curDF = .1;
	int dir = 1; // 1 or -1, direction to spin, + is for CCW - for CW
	float alpha = -0.8; // alpha should be negative, proportional gain on anticipated roll in order to cancel it
	float beta = -0.05;// negative, sort of like a weird spring.
	float midspring = -0.5;
	float gamma = 0.1;
	// Output from middle term should be positive infinity near zero, and negative infinity near pi. 0 at 0
	// gamma should be positive, damping constant
	// gamma might not be needed due to inherent damping in the system
	uint32_t tLast; // System time @ last velocity sample
	float limit = 0.7; // limit max df
	float tail_pos; 
	float roll_vel;
	float velocities [velocity_buffer];
	float roll_accels [roll_accel_buffer];
	//float roll_accel;
	float tail_vel;
	float tau;
	int cur_time;
	int dt;
	bool activate;
	float angDes;
	float tail_kin_params[1] = {0.5};
	
	void begin() {
		mode = TS_WAIT; //Start in wait mode
		posDes = 0; // Initialize desired position to zero
		tLast = S->millis;// Set tLast at onset 
		limb[4].setDims(1,1);
		P->limbs[4].jointInd_count = 1;
		P->limbs[4].jointInd[0] = 8; // tail joint index into the fifth limb
	}

	//sig is mapped from remote; here, 3 corresponds to pushing the left stick to the right
	// which in turn forces the state machine into TS_SPIN
	void signal(uint32_t sig)
	{
		if(sig == 2) mode = TS_SPIN; // start the spinning
		if(sig == 3) mode = TS_WAIT; // stop the spinning
	}

	void update() {
		C->mode = RobotCommand_Mode_LIMB;
		static float prev_roll_vel = 0;
		static float prev_tail_pos = joint[8].getPosition()+PI/2;
		static int v_count = 0;
		static bool filled_velocity_buffer = 0;
  		static float velocities [velocity_buffer];
  		static int roll_count = 0;
  		static bool filled_roll_accel_buffer = 0;
  		static float roll_accels [roll_accel_buffer];

  		cur_time = S->millis;
		
		//curDF = limb[4].getOpenLoop(0); // doesn't work inlimb mode
		tail_pos = joint[8].getPosition()+PI/2;

		if(mode == TS_WAIT){
			limb[4].setOpenLoop(0,0); //don't do anything
			tLast = S->millis;
		}
		else if(mode == TS_SPIN){

			// leg stuff
			for (int i = 0; i < 4; ++i){
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
				if (i == 0 || i == 1) {
					limb[i].setGain(ANGLE, 0.1, 0.03);
					limb[i].setPosition(ANGLE, 0);

					limb[i].setGain(EXTENSION, 0.1, 0.04);
					limb[i].setPosition(EXTENSION, 1 );
				}
				else {
					limb[i].setGain(ANGLE, 0.1, 0.03);
					limb[i].setPosition(ANGLE, 3.14);

					limb[i].setGain(EXTENSION, 0.1, 0.04);
					limb[i].setPosition(EXTENSION, 2 );
				}
			}


			//tail joint as limb

			//C->mode = RobotCommand_Mode_JOINT;
			// get body info
			roll_vel = S->imu.angular_velocity.y; // x is roll, y is pitch?
			tail_pos = limb[4].getPosition(0)+PI/2;
			if (tail_pos < 0) tail_pos = 0; // treat angles beyond 180 as if they were on the boundary
			else if (tail_pos > PI) tail_pos = PI;
			dt = cur_time - tLast;
			if(dt < 1){dt = 1;}

			roll_accels[roll_count] = (roll_vel - prev_roll_vel)/float(dt);
			velocities[v_count] = (tail_pos - prev_tail_pos)/float(dt);

			roll_count = (roll_count + 1);
			v_count = (v_count + 1);

			if (roll_count >= roll_accel_buffer){filled_roll_accel_buffer = 1;}
			if (v_count >= velocity_buffer){filled_velocity_buffer = 1;}
			
			roll_count = (roll_count) % roll_accel_buffer;
			v_count = (v_count) % velocity_buffer;

			if (filled_roll_accel_buffer){
				::roll_accel = avg(roll_accels, roll_accel_buffer);}
			else{
				::roll_accel = 0;
			}
			if (filled_velocity_buffer){
				tail_vel = avg(velocities, velocity_buffer);
			}
			else{
				tail_vel = velocities[v_count-1];
			}
			// if angular accel is opposite in direction to angular velocity, don't do anything
			// because something else is already correcting angular accel
			// if angular accel is zero and vel is non-zero, robot is spinning through space at constant speed
			//		don't use the tail
			// if angular accel is not zero and vel is zero, robot is moving in an undesirable way
			if (roll_accel * roll_vel < 0) {
				activate = 0;
			} else if (almostEq(roll_accel, 0) && !almostEq(roll_vel, 0)){
				activate = 0;
			} else if (!almostEq(roll_accel, 0) && almostEq(roll_vel, 0)){
				activate = 1;
			} else {activate = 1;}

			// calculate ideal tail power using logistic function
			//tau = activate*alpha*(roll_accel) + beta*logf(tail_pos/(PI-tail_pos)) + gamma*tail_vel;
			
			// alternatively, right the tail with simple PD stuff
			tau = activate*alpha*(roll_accel) + beta*logf(tail_pos/(PI-tail_pos))+ midspring*(tail_pos-PI/2) + gamma*tail_vel;


			if (tau>limit){ tau = limit;}
			else if (tau<-limit){tau = -limit;} 
			// move tail
			P->limbs[4].fkFun = (uint64_t) myKinematics;
			P->limbs[4].kinParams_count = 1;
			P->limbs[4].kinParams[0] = tail_kin_params[0];
			limb[4].setOpenLoop(0,tau);
			//limb[4].setOpenLoop(-0.1);
			// update timer
			tLast = cur_time;

		}
	}

	bool running() {
		return true;
	}

	void end() {
	}
};

int main(int argc, char *argv[]) {

	#if defined(ROBOT_MINITAUR)
	init(RobotParams_Type_MINITAUR, argc, argv);
	/*for (int i = 0; i < P->joints_count+1; ++i)
		P->joints[i].zero = motZeros[i]; //Add motor zeros from array at beginning of file*/
	#elif defined(ROBOT_MINITAUR_E)
	init(RobotParams_Type_MINITAUR_E, argc, argv);
	JoyType joyType = JoyType_FRSKY_XSR;
	ioctl(JOYSTICK_FILENO, IOCTL_CMD_JOYSTICK_SET_TYPE, &joyType);
	#else
	#error "Define robot type in preprocessor"
	#endif

	// Configure joints
	#define NUM_MOTORS 9
	//const float zeros[NUM_MOTORS] = {0, 0, 0, 0, 0, 0, 0, 0};
	const float directions[NUM_MOTORS] = {1, 1, 1, 1, 1, 1, 1, 1, 1};
	P->joints_count = S->joints_count = C->joints_count = NUM_MOTORS;
	for (int i = 0; i < P->joints_count; i++)
	{
		// Set zeros and directions
		P->joints[i].zero = motZeros[i];
		P->joints[i].direction = directions[i];
	}
	// Set the joint type; see JointParams
	P->joints[8].type = JointParams_Type_GRBL;

// Set the *physical* address (e.g. etherCAT ID, PWM port, dynamixel ID, etc.)
	P->joints[8].address = 8;

// If there is a gearbox the joint electronics doesn't know about, this could be > 1.
// Do not set to 0.
	P->joints[8].gearRatio = 1;
	P->limbs_count = 5; 
	TailSpin tailSpin; //Declare instance of our behavior
	//Disable the safety shut off feature:
	//IT IS POSSIBLE TO DAMAGE THE MOTOR; BE CAREFUL WHEN USING
	//BEHAVIORS WITHOUT THIS FAILSAFE 
	safetyShutoffEnable(false);
	//Disable the softStart feature
	softStartEnable(false);
	//Remove default behaviors from behaviors vector
	behaviors.clear();
	//add our behavior to behaviors vector
	behaviors.push_back(&tailSpin);

/*	SerialPortConfig cfg;
	cfg.baud = 115200;
	cfg.mode = SERIAL_8N1;
	ioctl(STDOUT_FILENO, IOCTL_CMD_SERIAL_PORT_CFG, &cfg);
	setDebugRate(2);*/
	setDebugRate(10);
	return begin();
}