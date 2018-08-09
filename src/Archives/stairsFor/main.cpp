/**
 * Modified from Ghost examples
 * 
 * Zhiyi (Allen) Ren, zhiyi.ren@jhu.edu
 * Stair climbing
 * 
 */
#include <stdio.h>
#include <SDK.h>
#include <math.h>
#include <Motor.h>
#include <ReorientableBehavior.h>

#include <Interpolator.h>
#include <func.h>

#include <string>
#include <iostream>

#if defined(ROBOT_MINITAUR)
// Subject to change for individual robots
// const float motZeros[8] = {2.82, 3.435, 3.54, 3.076, 1.03, 3.08, 6.190, 1.493};
// const float motZeros[8] = {2.570, 3.167, 3.777, 3.853, 2.183, 1.556, .675, 2.679}; // RML Ellie
const float motZeros[8] = {2.570, 3.190, 3.777, 3.853, 2.183, 1.556, .675, 2.679}; // RML Ellie corrected

// const float motZeros[8] = {0.631, 4.076, 1.852, 3.414, 1.817, 5.500, 1.078, 6.252}; //RML Odie
#endif

char myData[32];
float *myData_buf = (float*)myData;

// State machine representation of behavior
enum FHMode
{
	FH_SIT = 0,
	FH_STAND,
	FH_UP
};

class Stairs : public ReorientableBehavior
{
public:
	FHMode mode = FH_SIT; // Current state within state-machine

	// int t;
	// int t_left = 0;
	// int t_right = 0;
	// float step = 0.001;

	int ru_time = 0;
	int ru_state = 1;

	float rear_init_ang;

	// use joystick to start walking or stop
	void signal(uint32_t sig)
	{
		if (sig == 3)
		{
			mode = FH_UP;
		}
		else
		{
			mode = FH_STAND;
		}
	}
	void begin()
	{
		mode = FH_STAND;			// Start behavior in STAND mode		
		for(int i=0; i<4; i++) P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;
	}

	void update()
	{
		// ioctl(LOGGER_FILENO, 1); // start logging
		// C->mode = RobotCommand_Mode_JOINT;
		C->mode = RobotCommand_Mode_LIMB;

		if (mode == FH_STAND)
		{
			ioctl(LOGGER_FILENO, 0); // start logging

			for(int i=0; i<4; i++) P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;
			// ensure reset
			ru_time = 0;
			ru_state = 1;

			for(int i=0; i<4; i++)
			{
				float ang = (isFront(i)) ? -S->imu.euler.y - 0.10 : -S->imu.euler.y + 0.20;
				limb[i].setGain(ANGLE, 0.8, 0.02);
				limb[i].setPosition(ANGLE, ang);

				float ext = (isFront(i)) ? 0.17 : 0.23;
				limb[i].setGain(EXTENSION, 120, 4);
				limb[i].setPosition(EXTENSION, ext);
			}
			// rear_init_ang = -S->imu.euler.y + (limb[1].getPosition(ANGLE)+limb[3].getPosition(ANGLE));
		}
		else if (mode == FH_UP)
		{
			if(ru_state == 1)  // front hold, rear retract for jump
			{
				ioctl(LOGGER_FILENO, 1); // start logging

				// front
				limb[0].setGain(ANGLE, 1.4, 0.02);
				limb[0].setPosition(ANGLE, -S->imu.euler.y - 0.10);
				limb[2].setGain(ANGLE, 1.4, 0.02);
				limb[2].setPosition(ANGLE, -S->imu.euler.y - 0.10);

				P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
				P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
				limb[0].setOpenLoop(EXTENSION, 0.10);
				limb[2].setOpenLoop(EXTENSION, 0.10);
				// rear
				limb[1].setGain(ANGLE, 1.4, 0.02);
				limb[1].setPosition(ANGLE, -S->imu.euler.y + 0.20);
				limb[3].setGain(ANGLE, 1.4, 0.02);
				limb[3].setPosition(ANGLE, -S->imu.euler.y + 0.20);

				P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[1].setGain(EXTENSION, 100, 3);	// retract, low gain for slower
				limb[1].setPosition(EXTENSION, 0.18);
				limb[3].setGain(EXTENSION, 100, 3); 
				limb[3].setPosition(EXTENSION, 0.18);
				// check
				if(limb[1].getPosition(EXTENSION) < 0.185 &&
					limb[3].getPosition(EXTENSION) < 0.185) ru_state = 2;
			}
			else if(ru_state == 2)  // front hold, rear extend
			{
				ioctl(LOGGER_FILENO, 1); // start logging

				// front
				limb[0].setGain(ANGLE, 1.4, 0.02);
				limb[0].setPosition(ANGLE, -S->imu.euler.y - 0.10);
				limb[2].setGain(ANGLE, 1.4, 0.02);
				limb[2].setPosition(ANGLE, -S->imu.euler.y - 0.10);

				P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
				P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
				limb[0].setOpenLoop(EXTENSION, 0.10);
				limb[2].setOpenLoop(EXTENSION, 0.10);
				// rear
				limb[1].setGain(ANGLE, 1.4, 0.02);
				limb[1].setPosition(ANGLE, -S->imu.euler.y + 0.30);
				limb[3].setGain(ANGLE, 1.4, 0.02);
				limb[3].setPosition(ANGLE, -S->imu.euler.y + 0.30);

				P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
				P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
				float comp = 0.1*(limb[1].getPosition(EXTENSION) - limb[3].getPosition(EXTENSION));
				if(comp <= 0)
				{
					limb[1].setOpenLoop(EXTENSION, 1.6);
					limb[3].setOpenLoop(EXTENSION, 1.6+comp);
				}
				else
				{
					limb[1].setOpenLoop(EXTENSION, 1.6-comp);  // push ground
					limb[3].setOpenLoop(EXTENSION, 1.6);					
				}
				// check
				if(limb[1].getPosition(EXTENSION) > 2.7 &&
					limb[3].getPosition(EXTENSION) > 2.7) ru_state = 3;
			}
			else if(ru_state == 3)  // front hold, rear retract/jump
			{
				ioctl(LOGGER_FILENO, 1); // start logging

				// front, was -0.1
				// if(limb[0].getPosition(ANGLE) < -S->imu.euler.y - 0.35)  // can't really reach
				if((++ru_time) < 50)
				{
					limb[0].setGain(ANGLE, 1.6, 0.03);
					limb[0].setPosition(ANGLE, -S->imu.euler.y - 0.40);
					limb[2].setGain(ANGLE, 1.6, 0.03);
					limb[2].setPosition(ANGLE, -S->imu.euler.y - 0.40);

					P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
					P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
					limb[0].setOpenLoop(EXTENSION, 0.05);
					limb[2].setOpenLoop(EXTENSION, 0.05);
				}
				else
				{
					limb[0].setGain(ANGLE, 1.8, 0.03);
					limb[0].setPosition(ANGLE, -S->imu.euler.y + 0.2);
					limb[0].setGain(ANGLE, 1.8, 0.03);
					limb[2].setPosition(ANGLE, -S->imu.euler.y + 0.2);

					// limb[0].setOpenLoop(EXTENSION, 0.30);  // rock forward
					// limb[2].setOpenLoop(EXTENSION, 0.30);
					P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
					P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
					limb[1].setGain(EXTENSION, 120, 4);
					limb[1].setPosition(EXTENSION, 0.11);
					limb[3].setGain(EXTENSION, 120, 4);
					limb[3].setPosition(EXTENSION, 0.11);
				}

				// rear
				limb[1].setGain(ANGLE, 1.4, 0.02);
				limb[1].setPosition(ANGLE, -S->imu.euler.y + 0.30);
				limb[3].setGain(ANGLE, 1.4, 0.02);
				limb[3].setPosition(ANGLE, -S->imu.euler.y + 0.30);

				P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[1].setGain(EXTENSION, 180, 5);
				limb[1].setPosition(EXTENSION, 0.11);
				limb[3].setGain(EXTENSION, 180, 5);
				limb[3].setPosition(EXTENSION, 0.11);
				// check
				if(limb[1].getPosition(EXTENSION) < 0.115 &&
					limb[3].getPosition(EXTENSION) < 0.115) 
				{
					ru_state = 4;
					ru_time = 0;
				}
			}
			else if(ru_state == 4)  // front move forward, rear move forward in air to TD
			{
				ioctl(LOGGER_FILENO, 1); // start logging
	
				// front
				limb[0].setGain(ANGLE, 1.4, 0.02);
				limb[0].setPosition(ANGLE, -S->imu.euler.y + 0.20);
				limb[2].setGain(ANGLE, 1.4, 0.02);
				limb[2].setPosition(ANGLE, -S->imu.euler.y + 0.20);

				P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[0].setGain(EXTENSION, 140, 4);
				limb[0].setPosition(EXTENSION, 0.15);
				limb[2].setGain(EXTENSION, 140, 4);
				limb[2].setPosition(EXTENSION, 0.15);
				// P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
				// P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
				// limb[0].setOpenLoop(EXTENSION, 0.15);  // anchor there?
				// limb[2].setOpenLoop(EXTENSION, 0.15);
				// rear
				limb[1].setGain(ANGLE, 1.6, 0.02);
				limb[1].setPosition(ANGLE, -S->imu.euler.y - 0.6);
				limb[3].setGain(ANGLE, 1.6, 0.02);
				limb[3].setPosition(ANGLE, -S->imu.euler.y - 0.6);

				P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[1].setGain(EXTENSION, 160, 5);
				limb[1].setPosition(EXTENSION, 0.15);
				limb[3].setGain(EXTENSION, 160, 5);
				limb[3].setPosition(EXTENSION, 0.15);
				// check
				if((++ru_time) > 200)
				{
					ru_state = 5;
					ru_time = 0;
				}
			}
			else if(ru_state == 5)  // balance
			{
				ioctl(LOGGER_FILENO, 1); // start logging
	
				// front
				limb[0].setGain(ANGLE, 1.2, 0.02);
				limb[0].setPosition(ANGLE, -S->imu.euler.y + 0);
				limb[2].setGain(ANGLE, 1.2, 0.02);
				limb[2].setPosition(ANGLE, -S->imu.euler.y + 0);

				P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[0].setGain(EXTENSION, 160, 5);
				limb[0].setPosition(EXTENSION, 0.16);  // anchor
				limb[2].setGain(EXTENSION, 160, 5);
				limb[2].setPosition(EXTENSION, 0.16);
				// rear
				limb[1].setGain(ANGLE, 1.2, 0.02);
				limb[1].setPosition(ANGLE, -S->imu.euler.y - 0.2);
				limb[3].setGain(ANGLE, 1.2, 0.02);
				limb[3].setPosition(ANGLE, -S->imu.euler.y - 0.2);

				P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[1].setGain(EXTENSION, 140, 4);
				limb[1].setPosition(EXTENSION, 0.16);
				limb[3].setGain(EXTENSION, 140, 4);
				limb[3].setPosition(EXTENSION, 0.16);
				// check
				if((++ru_time) > 200)
				{
					ru_state = 5;
					ru_time = 0;
				}
			}
			// else if(ru_state == 5)  // front move forward, rear move forward in air to TD
			// {
			// 	ioctl(LOGGER_FILENO, 1); // start logging
	
			// 	// front
			// 	limb[0].setGain(ANGLE, 1.6, 0.02);
			// 	limb[0].setPosition(ANGLE, -S->imu.euler.y + 0.30);
			// 	limb[2].setGain(ANGLE, 1.6, 0.02);
			// 	limb[2].setPosition(ANGLE, -S->imu.euler.y + 0.30);

			// 	P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
			// 	P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
			// 	limb[0].setGain(EXTENSION, 200, 4);
			// 	limb[0].setPosition(EXTENSION, limb[0].getPosition(EXTENSION));  // anchor
			// 	limb[2].setGain(EXTENSION, 200, 4);
			// 	limb[2].setPosition(EXTENSION, limb[2].getPosition(EXTENSION));
			// 	// rear
			// 	limb[1].setGain(ANGLE, 1.6, 0.02);
			// 	limb[1].setPosition(ANGLE, -S->imu.euler.y - 0.2);
			// 	limb[3].setGain(ANGLE, 1.6, 0.02);
			// 	limb[3].setPosition(ANGLE, -S->imu.euler.y - 0.2);

			// 	P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
			// 	P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
			// 	limb[1].setGain(EXTENSION, 180, 4);
			// 	limb[1].setPosition(EXTENSION, 0.18);
			// 	limb[3].setGain(EXTENSION, 180, 4);
			// 	limb[3].setPosition(EXTENSION, 0.18);
			// 	// check
			// 	if((++ru_time) > 200)
			// 	{
			// 		ru_state = 6;
			// 		ru_time = 0;
			// 	}
			// }
			// else if(ru_state == 6)  // front extend, rear hold
			// {
			// 	ioctl(LOGGER_FILENO, 0);

			// 	// front
			// 	limb[0].setGain(ANGLE, 1.4, 0.02);
			// 	limb[0].setPosition(ANGLE, -S->imu.euler.y + 0.20);
			// 	limb[2].setGain(ANGLE, 1.4, 0.02);
			// 	limb[2].setPosition(ANGLE, -S->imu.euler.y + 0.20);

			// 	P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
			// 	P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
			// 	float comp = 0.1*(limb[0].getPosition(EXTENSION) - limb[2].getPosition(EXTENSION));
			// 	if(comp <= 0)
			// 	{
			// 		limb[0].setOpenLoop(EXTENSION, 1.0);
			// 		limb[2].setOpenLoop(EXTENSION, 1.0+comp);
			// 	}
			// 	else{
			// 		limb[0].setOpenLoop(EXTENSION, 1.0-comp);  // push ground
			// 		limb[2].setOpenLoop(EXTENSION, 1.0);					
			// 	}
			// 	// rear
			// 	limb[1].setGain(ANGLE, 1.4, 0.02);
			// 	limb[1].setPosition(ANGLE, -S->imu.euler.y - 0.2);
			// 	limb[3].setGain(ANGLE, 1.4, 0.02);
			// 	limb[3].setPosition(ANGLE, -S->imu.euler.y - 0.2);

			// 	P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
			// 	P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
			// 	limb[1].setGain(EXTENSION, 180, 4);
			// 	limb[1].setPosition(EXTENSION, 0.20);
			// 	limb[3].setGain(EXTENSION, 180, 4);
			// 	limb[1].setPosition(EXTENSION, 0.20);
			// 	// check
			// 	if(limb[0].getPosition(EXTENSION) > 2.6 &&
			// 		limb[2].getPosition(EXTENSION) > 2.6) ru_state = 7;	
			// }
			// else if(ru_state == 7)  // front retract/jump, rear hold
			// {
			// 	ioctl(LOGGER_FILENO, 0);

			// 	// front
			// 	limb[0].setGain(ANGLE, 1.4, 0.02);
			// 	limb[0].setPosition(ANGLE, -S->imu.euler.y + 0.30);
			// 	limb[2].setGain(ANGLE, 1.4, 0.02);
			// 	limb[2].setPosition(ANGLE, -S->imu.euler.y + 0.30);

			// 	P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
			// 	P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
			// 	limb[0].setGain(EXTENSION, 200, 4);
			// 	limb[0].setPosition(EXTENSION, 0.11);
			// 	limb[2].setGain(EXTENSION, 200, 4);
			// 	limb[2].setPosition(EXTENSION, 0.11);
			// 	// rear
			// 	limb[1].setGain(ANGLE, 1.4, 0.02);
			// 	limb[1].setPosition(ANGLE, -S->imu.euler.y - 0.2);
			// 	limb[3].setGain(ANGLE, 1.4, 0.02);
			// 	limb[3].setPosition(ANGLE, -S->imu.euler.y - 0.2);

			// 	P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
			// 	P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
			// 	// limb[1].setGain(EXTENSION, 180, 4);
			// 	// limb[1].setPosition(EXTENSION, 0.20);
			// 	// limb[3].setGain(EXTENSION, 180, 4);
			// 	// limb[3].setPosition(EXTENSION, 0.20);
			// 	limb[1].setOpenLoop(EXTENSION, 0.08);
			// 	limb[3].setOpenLoop(EXTENSION, 0.08);
			// 	// check
			// 	if(limb[0].getPosition(EXTENSION) < 0.115 &&
			// 		limb[2].getPosition(EXTENSION) < 0.115) ru_state = 8;
			// }
			// else if(ru_state == 8)  // front move forward in air to TD, rear hold
			// {
			// 	ioctl(LOGGER_FILENO, 0);

			// 	// front
			// 	limb[0].setGain(ANGLE, 1.2, 0.02);
			// 	limb[0].setPosition(ANGLE, -S->imu.euler.y - 0.5);
			// 	limb[2].setGain(ANGLE, 1.2, 0.02);
			// 	limb[2].setPosition(ANGLE, -S->imu.euler.y - 0.5);

			// 	P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
			// 	P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
			// 	limb[0].setGain(EXTENSION, 140, 4);
			// 	limb[0].setPosition(EXTENSION, 0.15);
			// 	limb[2].setGain(EXTENSION, 140, 4);
			// 	limb[2].setPosition(EXTENSION, 0.15);
			// 	// rear
			// 	limb[1].setGain(ANGLE, 1.4, 0.02);
			// 	limb[1].setPosition(ANGLE, -S->imu.euler.y + 0.1);
			// 	limb[3].setGain(ANGLE, 1.4, 0.02);
			// 	limb[3].setPosition(ANGLE, -S->imu.euler.y + 0.1);

			// 	P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
			// 	P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
			// 	limb[1].setGain(EXTENSION, 180, 4);
			// 	limb[1].setPosition(EXTENSION, 0.21);
			// 	limb[3].setGain(EXTENSION, 180, 4);
			// 	limb[3].setPosition(EXTENSION, 0.21);
			// 	// check
			// 	if((++ru_time) > 120)
			// 	{
			// 		ru_state = 9;
			// 		ru_time = 0;
			// 	}		
			// }
			// else if(ru_state == 9)  // balance until front leg tilt forward
			// {
			// 	ioctl(LOGGER_FILENO, 0);

			// 	// front
			// 	limb[0].setGain(ANGLE, 1.4, 0.02);
			// 	limb[0].setPosition(ANGLE, -S->imu.euler.y - 0.2);
			// 	limb[2].setGain(ANGLE, 1.4, 0.02);
			// 	limb[2].setPosition(ANGLE, -S->imu.euler.y - 0.2);

			// 	P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
			// 	P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
			// 	limb[0].setGain(EXTENSION, 140, 4);
			// 	limb[0].setPosition(EXTENSION, 0.17);
			// 	limb[2].setGain(EXTENSION, 140, 4);
			// 	limb[2].setPosition(EXTENSION, 0.17);

			// 	// rear
			// 	limb[1].setGain(ANGLE, 1.4, 0.02);
			// 	limb[1].setPosition(ANGLE, -S->imu.euler.y + 0.1);
			// 	limb[3].setGain(ANGLE, 1.4, 0.02);
			// 	limb[3].setPosition(ANGLE, -S->imu.euler.y + 0).1;

			// 	P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
			// 	P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
			// 	limb[1].setGain(EXTENSION, 160, 4);
			// 	limb[1].setPosition(EXTENSION, 0.22);
			// 	limb[3].setGain(EXTENSION, 160, 4);
			// 	limb[3].setPosition(EXTENSION, 0.22);
			// 	// check
			// 	if(limb[0].getPosition(ANGLE) > -S->imu.euler.y - 0.25 &&
			// 		limb[2].getPosition(ANGLE) > -S->imu.euler.y - 0.25) ru_state = 10;
			// }
			else if(ru_state == 10)  // reset, back to stand
			{
				ioctl(LOGGER_FILENO, 0);
				ru_state = 1;
				mode = FH_STAND;
			}
		}
			// print_var = ru_state;
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

void debug()
{
	// myData_buf[0] = limb[0].getForce(EXTENSION);
	// myData_buf[1] = limb[1].getForce(EXTENSION);
	// myData_buf[2] = limb[2].getForce(EXTENSION);
	// myData_buf[3] = limb[3].getForce(EXTENSION);
	// myData_buf[4] = limb[0].getForce(ANGLE);
	// myData_buf[5] = limb[1].getForce(ANGLE);
	// myData_buf[6] = limb[2].getForce(ANGLE);
	// myData_buf[7] = limb[3].getForce(ANGLE);

	// write(LOGGER_FILENO, myData, 32);

	// printf("State: %f.\n", print_var);
}

int main(int argc, char *argv[])
{
	// MINITAUR
	init(RobotParams_Type_MINITAUR, argc, argv);
	for (int i = 0; i < P->joints_count; ++i)
		P->joints[i].zero = motZeros[i];

	// for SD card logging
  setDebugRate(100);

	Stairs stairs;
	behaviors.push_back(&stairs);

	return begin();
}
