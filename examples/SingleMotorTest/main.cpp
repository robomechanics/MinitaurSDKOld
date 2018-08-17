/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Gavin Kenneally, Avik De, Turner Topping <gavin@ghostrobotics.io> <avik@ghostrobotics.io> <turner@ghostrobotics.io>
 */
#include <stdio.h>
#include <SDK.h>
#include <math.h>
#include <Smath.h>
#include <Motor.h>
#include <Behavior.h>

#define MOT 9

char myData[32];
float* myData_buf = (float*)myData;

enum SMTMode {
	SMT_STOP = 0, SMT_GO
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

const int velocityBuffer = 1000;
static float driverVelocities [velocityBuffer];
static int counter = 0;
static bool filledUpArray = 0;

class SingleMotorTest : public Behavior {
public:
	SMTMode mode; //Current state within state-machine

	float posDes; //Desired position
	float kp = 0.5;
	float kd = 0;

	// If using setOpenLoop rather than setPosition
	float velDes; //Desired velocity
	float cmd;
	float cmdMax = 1;

	uint32_t tLast; // System time @ last state change

    float betterDriverVelocity;
    float avgVel = 0;
	
	void begin() {
		mode = SMT_GO;
		avgVel = 0;
		counter = 0;
		tLast = S->millis;// Set tLast at onset 
	}

	void update() {
		C->mode = RobotCommand_Mode_JOINT;

		if (mode == SMT_GO)
		{
			// joint[MOT].setGain(kp, kd); //Sets P and D gains for position based control
			posDes = 0;
			// joint[MOT].setPosition(posDes);
			cmd = 0.1*((int)((S->millis - tLast) / 5000) % 10);
			joint[MOT].setOpenLoop(cmd);
			// joint[MOT].setOpenLoop(1/(0.95*16)*(0.05*0.2/0.0954 + 0.0954*1*joint[MOT].getVelocity()));

			// posDes = 0;
			// velDes = 0;
			// cmd = kp*(posDes - joint[MOT].getPosition()) + kd*(velDes - joint[MOT].getVelocity());
			// cmd = constrain(cmd, -cmdMax, cmdMax);
			// joint[MOT].setOpenLoop(cmd);
		}
		else
		{
			joint[MOT].setOpenLoop(0.0);
		}

        if (S->millis - tLast >=250)
        {
        	driverVelocities[counter] = joint[MOT].getVelocity();

        	if(counter == velocityBuffer-1) filledUpArray = 1; //now we can average the posVelocities
            counter++;
      		if (filledUpArray == 1){
        	    betterDriverVelocity = avg(driverVelocities, velocityBuffer);
        	    avgVel = betterDriverVelocity;

            		counter = 0;
            	filledUpArray = 0;

       		}
        }



	}

	bool running() {
		return false;
	}

	void end() {
		mode = SMT_STOP;
	}
};

SingleMotorTest singleMotorTest; //Declare instance of our behavior

void debug()
{
	printf("Time: %4.1fs. ", (float) 0.001*(S->millis - singleMotorTest.tLast));
	printf("Mode: %d, ", singleMotorTest.mode);
	// printf("Pos: %6.3f, ", joint[MOT].getPosition());
	printf("Vel: %6.3f, ", joint[MOT].getVelocity());
	printf("AvgVel: %6.3f, ", singleMotorTest.avgVel);
	
	// printf("Raw Pos: %6.3f, ",joint[MOT].getRawPosition());
	printf("Command: %6.3f. ", joint[MOT].getOpenLoop());
	printf("\n");

	// myData_buf[0] = joint[MOT].getPosition();
	// myData_buf[1] = joint[MOT].getVelocity();
	// myData_buf[2] = joint[MOT].getOpenLoop();
	// write(LOGGER_FILENO, myData, 32);
}

int main(int argc, char *argv[]) {
	init(RobotParams_Type_MINITAUR, argc, argv);

	// Set the joint type; see JointParams
	P->joints[MOT].type = JointParams_Type_GRBL;

	// Set the *physical* address (e.g. etherCAT ID, PWM port, dynamixel ID, etc.)
	P->joints[MOT].address = MOT;

	// If there is a gearbox the joint electronics doesn't know about, this could be > 1.
	// Do not set to 0.
	P->joints[MOT].gearRatio = 1.0;
	// P->joints[MOT].gearRatio = 45.39;

	// Configure joints
	#define NUM_MOTORS 10
	const float zeros[NUM_MOTORS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	const float directions[NUM_MOTORS] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
	P->joints_count = S->joints_count = C->joints_count = NUM_MOTORS;
	for (int i = 0; i < P->joints_count; i++)
	{
		// Set zeros and directions
		P->joints[i].zero = zeros[i];
		P->joints[i].direction = directions[i];
	}

	P->limbs_count = 0;

	//Disable the safety shut off feature:
	//IT IS POSSIBLE TO DAMAGE THE MOTOR; BE CAREFUL WHEN USING
	//BEHAVIORS WITHOUT THIS FAILSAFE 
	safetyShutoffEnable(false);
	//Disable the softStart feature
	softStartEnable(false);
	//Remove default behaviors from behaviors vector
	behaviors.clear();
	//add our behavior to behaviors vector
	behaviors.push_back(&singleMotorTest);

	setDebugRate(10);

	SerialPortConfig cfg;
	cfg.baud = 115200;
	cfg.mode = SERIAL_8N1;
	ioctl(STDOUT_FILENO, IOCTL_CMD_SERIAL_PORT_CFG, &cfg);

	return begin();
}

