/**
 * Modified from Ghost examples
 * 
 * Zhiyi (Allen) Ren, zhiyi.ren@jhu.edu
 * Stair climbing, use limb 1 and 3 as front ones, toe facing backwards then for stability
 * 
 */
#include <stdio.h>
#include <SDK.h>
#include <math.h>
#include <Motor.h>
#include <ReorientableBehavior.h>

#include <Interpolator.h>
#include <func.h>
#include <observer.h>

#include <string>
#include <iostream>

#if defined(ROBOT_MINITAUR)
// Subject to change for individual robots
const float motZeros[8] = {2.570, 2.036, 3.777, 3.853, 2.183, 1.556, .675, 2.679}; // RML Ellie
// const float motZeros[8] = {0.631, 4.076, 1.852, 3.414, 1.817, 5.500, 1.078, 6.252}; //RML Odie
#endif

char myData[32];
float *myData_buf = (float*)myData;

// State machine representation of behavior
enum FHMode
{
	FH_SIT = 0,
	FH_STAND,
	FH_FU,
	FH_RU,
	FH_RECOV
};

// float pos_int[8] = {0,0,0,0,0,0,0,0};
// float vel_int[8] = {0,0,0,0,0,0,0,0};
float pos_act_old[8] = {0,0,0,0,0,0,0,0};
// float vel_act_old[8] = {0,0,0,0,0,0,0,0};
float pos_act[8] = {0,0,0,0,0,0,0,0};
float vel_act[8] = {0,0,0,0,0,0,0,0};
float pos_obs_int[8] = {0,0,0,0,0,0,0,0};
float vel_obs_int[8] = {0,0,0,0,0,0,0,0};
float pos_obs[8] = {0,0,0,0,0,0,0,0};
float vel_obs[8] = {0,0,0,0,0,0,0,0};

float pos_obs_old[8] = {0,0,0,0,0,0,0,0};
float vel_obs_old[8] = {0,0,0,0,0,0,0,0};

float res_pos[8] = {0,0,0,0,0,0,0,0};
float res_vel[8] = {0,0,0,0,0,0,0,0};
float res_pos_avg[4] = {0,0,0,0};  // four limbs
float res_vel_avg[4] = {0,0,0,0};

// test time
int t1 = 0;
int t2 = 0;
int nowTime = 0;

float pos_obs_seq[4] = {0,0,0,0};
float pos_act_seq[4] = {0,0,0,0};

float checkPWM[2] = {0,0};
int obsTimes = -1;
int obsNowTime = -1;

class Stairs : public ReorientableBehavior
{
public:
	FHMode mode = FH_SIT; // Current state within state-machine

	int t;
	int t_left = 0;
	int t_right = 0;
	float step = 0.001;

	bool start = true;
	int startTime = 0;

	// recovery
	// bool hit[4] = {false, false, false, false};
	// bool miss[4] = {false, false, false, false};
	// float res[8];
	// int recov_time[4] = {0,0,0,0};

	int recov_state = 1;
	int recov_time = 0;
	bool recov_start = true;

	// rear jump
	int ru_time = 0;
	int ru_state = 1;
	// front jump
	int fu_time = 0;
	int fu_state = 1;
	float fu_pwm = 0.8;  // start with 0.4 PWM

	// trotting, gains for trotting
	float integral[8] = {0,0,0,0,0,0,0,0};
	float pwm[8];
	float kpl = 0.3;
	float kpm = 0.6;
	float kph = 0.9;
	float kd = 0.01;
	float ki = 0.1;

	float PWM0 = 0;
	float PWM1 = 0;
	float PWM4 = 0;
	float PWM5 = 0;

	// observer
	float batt_act;
	bool first = true;  // for testing time
	int lastTime = 0;
	// int obsCount = 0;

	// use joystick to start walking or stop
	void signal(uint32_t sig)
	{
		if (sig == 3)
		{
			mode = FH_RU;
		}
		else
		{
			mode = FH_STAND;
			// mode = FH_START;
		}
	}
	void begin()
	{
		ioctl(LOGGER_FILENO, 0); // do not log
		mode = FH_STAND;			// Start behavior in STAND mode

		// apExt, tdExt, angle, t_side, t_bottom_ period, offset
		// gW = {0.12, 0.18, -15, 100, 300, 800, 400};
		// sW = {0.13, 0.18, -5,  100, 200, 600, 300};

		// getTrajInterp(qIn, qOut, gW.apExt, gW.tdExt, gW.angle, gW.t_bottom, gW.t_side);  // get traj on ground

	 	// getTrajRotInterp(qInS, qOutS, sW.apExt, sW.tdExt, sW.angle, sW.t_bottom, sW.t_side);  // get traj on stair before jumping up, reuse same array

		// for(int i=0; i<4; i++) P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;
	}

	// STAND, GW, FIRST, SW, FU, RU, SW, FU, RU...
	void update()
	{
		if (mode == FH_STAND)
		{
			C->mode = RobotCommand_Mode_LIMB;
			ioctl(LOGGER_FILENO, 0); // do not logging

			for(int i=0; i<4; i++) P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;
			// ensure reset
			ru_time = 0;
			ru_state = 1;

			for(int i=0; i<4; i++)
			{
				float ang = (!isFront(i)) ? -S->imu.euler.y + 0 : -S->imu.euler.y - 0.10;
				limb[i].setGain(ANGLE, 0.8, 0.02);
				limb[i].setPosition(ANGLE, ang);

				// float ext = (!isFront(i)) ? 0.17 : 0.24;
				float ext = (!isFront(i)) ? 0.18 : 0.26;
				limb[i].setGain(EXTENSION, 120, 4);
				limb[i].setPosition(EXTENSION, ext);
			}
		}

		else if (mode == FH_RU)
		{
			// ioctl(LOGGER_FILENO, 0); // do not log
			C->mode = RobotCommand_Mode_LIMB;

			if(ru_state == 1)  // front hold, rear retract for jump
			{
				// ioctl(LOGGER_FILENO, 1); // start logging
				ioctl(LOGGER_FILENO, 0); // do not log
				// front
				limb[1].setGain(ANGLE, 1.2, 0.02);
				limb[1].setPosition(ANGLE, -S->imu.euler.y - 0);
				limb[3].setGain(ANGLE, 1.2, 0.02);
				limb[3].setPosition(ANGLE, -S->imu.euler.y - 0);

				P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
				P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
				limb[1].setOpenLoop(EXTENSION, 0.20);  // higher now
				limb[3].setOpenLoop(EXTENSION, 0.20);
				// rear
				limb[0].setGain(ANGLE, 1.4, 0.02);
				limb[0].setPosition(ANGLE, -S->imu.euler.y - 0.2);
				limb[2].setGain(ANGLE, 1.4, 0.02);
				limb[2].setPosition(ANGLE, -S->imu.euler.y - 0.2);

				P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[0].setGain(EXTENSION, 100, 3);	 // retract, low gain for slower
				limb[0].setPosition(EXTENSION, 0.24);
				limb[2].setGain(EXTENSION, 100, 3); 
				limb[2].setPosition(EXTENSION, 0.24);
				// check
				if(limb[0].getPosition(EXTENSION) < 0.245 &&
					limb[2].getPosition(EXTENSION) < 0.245) ru_state = 2;
			}
			else if(ru_state == 2)  // front hold, rear extend
			{
				// ioctl(LOGGER_FILENO, 1); // start logging
				ioctl(LOGGER_FILENO, 0); // do not log
				// front
				limb[1].setGain(ANGLE, 1.2, 0.02);
				limb[1].setPosition(ANGLE, -S->imu.euler.y - 0);
				limb[3].setGain(ANGLE, 1.2, 0.02);
				limb[3].setPosition(ANGLE, -S->imu.euler.y - 0);

				P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
				P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
				limb[1].setOpenLoop(EXTENSION, 0.20);
				limb[3].setOpenLoop(EXTENSION, 0.20);
				// rear
				limb[0].setGain(ANGLE, 1.4, 0.02);
				limb[0].setPosition(ANGLE, -S->imu.euler.y - 0.30);
				limb[2].setGain(ANGLE, 1.4, 0.02);
				limb[2].setPosition(ANGLE, -S->imu.euler.y - 0.30);

				P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
				P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
				float comp = 0.1*(limb[0].getPosition(EXTENSION) - limb[2].getPosition(EXTENSION));
				if(comp <= 0)
				{
					limb[0].setOpenLoop(EXTENSION, 1.0);
					limb[2].setOpenLoop(EXTENSION, 1.0+comp);
				}
				else
				{
					limb[0].setOpenLoop(EXTENSION, 1.0-comp);  // push ground
					limb[2].setOpenLoop(EXTENSION, 1.0);					
				}

				// check
				if(limb[0].getPosition(EXTENSION) > 2.7 &&
					limb[2].getPosition(EXTENSION) > 2.7) ru_state = 3;
			}
			else if(ru_state == 3)  // front hold, rear retract/jump
			{
				ioctl(LOGGER_FILENO, 1); // start logging

				if(start)
				{
					startTime = S->millis;
					start = false;
				}
				if(S->millis - lastTime < 5) return;

				if((++ru_time) < 50)
				{
					limb[1].setGain(ANGLE, 1.6, 0.03);
					limb[1].setPosition(ANGLE, -S->imu.euler.y + 0.30);
					limb[3].setGain(ANGLE, 1.6, 0.03);
					limb[3].setPosition(ANGLE, -S->imu.euler.y + 0.30);

					P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
					P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
					limb[1].setOpenLoop(EXTENSION, 0.10);
					limb[3].setOpenLoop(EXTENSION, 0.10);
				}
				else  // rock forward
				{
					limb[1].setGain(ANGLE, 1.8, 0.03);
					limb[1].setPosition(ANGLE, -S->imu.euler.y - 0);
					limb[3].setGain(ANGLE, 1.8, 0.03);
					limb[3].setPosition(ANGLE, -S->imu.euler.y - 0);

					P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
					P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
					limb[1].setOpenLoop(EXTENSION, 0.30);
					limb[3].setOpenLoop(EXTENSION, 0.30);
				}
				// rear
				limb[0].setGain(ANGLE, 1.2, 0.02);  // was 1.4 before
				limb[0].setPosition(ANGLE, -S->imu.euler.y - 0.30);
				limb[2].setGain(ANGLE, 1.2, 0.02);
				limb[2].setPosition(ANGLE, -S->imu.euler.y - 0.30);

				P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[0].setGain(EXTENSION, 180, 5);
				limb[0].setPosition(EXTENSION, 0.11);
				limb[2].setGain(EXTENSION, 180, 5);
				limb[2].setPosition(EXTENSION, 0.11);

				batt_act = S->batt.voltage;
				// batt_act = 16.0;
				for(int i=0; i<8; i++)
				{
					pos_obs[i] = pos_obs_int[i];
					vel_obs[i] = vel_obs_int[i];
				}

				// get new value
				pos_obs_int[0] = toT1(joint[1].getPosition());
				pos_obs_int[1] = toT2(joint[0].getPosition());
				pos_obs_int[4] = toT1(joint[4].getPosition());
				pos_obs_int[5] = toT2(joint[5].getPosition());

				vel_obs_int[0] = joint[1].getVelocity();
				vel_obs_int[1] = -joint[0].getVelocity();
				vel_obs_int[4] = joint[4].getVelocity();
				vel_obs_int[5] = -joint[5].getVelocity();	

				for(int i=0; i<8; i++)
				{
					pos_act_old[i] = pos_act[i];  // prev state
					// vel_act_old[i] = vel_act[i];

					pos_act[i] = pos_obs_int[i];
					vel_act[i] = vel_obs_int[i];
				}
				// calculate residual, int has the current value, obs has future value from previous step
				for(int i=0; i<8; i++)
				{
					res_pos[i] = fmodf_mpi_pi(pos_obs[i] - pos_obs_int[i]);
					res_vel[i] = vel_obs[i] - vel_obs_int[i];
				}
				for(int i=0; i<4; i++)
				{
					res_pos_avg[i] = (abs(res_pos[i*2])+abs(res_pos[i*2+1]))*0.5;
					res_vel_avg[i] = (abs(res_vel[i*2])+abs(res_vel[i*2+1]))*0.5;
				}

				// int index = obsCount % 4;
				// if((++obsCount) >= 4) obsCount = 0;
				// pos_obs_seq[index] = pos_obs[0];
				// pos_act_seq[index] = pos_act[0];

				// step: 0.002s, use the previous PWM, in theta coordinates
				obs(pos_obs_int, vel_obs_int, batt_act, PWM0, PWM1, 0, 0.002);
				obs(pos_obs_int, vel_obs_int, batt_act, PWM4, PWM5, 2, 0.002);
				// obs_ver(pos_obs_int, vel_obs_int, batt_act, PWM0, PWM1, 0, 0.002, pos_act_old);
				// obs_ver(pos_obs_int, vel_obs_int, batt_act, PWM4, PWM5, 2, 0.002, pos_act_old);

				// back limbs (0 and 2), joint 0/1 and 4/5
				PWM0 = joint[1].getOpenLoop();
				PWM1 = -joint[0].getOpenLoop();
				PWM4 = joint[4].getOpenLoop();
				PWM5 = -joint[5].getOpenLoop();

				// obsNowTime = S->millis - startTime;
				// obsTimes++;

				lastTime = S->millis;

				// check
				if(limb[0].getPosition(EXTENSION) < 0.115 &&
					limb[2].getPosition(EXTENSION) < 0.115) 
				{
					ru_state = 4;
					ru_time = 0;
					// t1 = S->millis - startTime;
				}
			}
			else if(ru_state == 4)  // front move forward, rear move forward in air to TD
			{
				ioctl(LOGGER_FILENO, 1); // start logging
				// nowTime = S->millis - startTime;
				if(start)
				{
					startTime = S->millis;
					start = false;
				}
				if(S->millis - lastTime < 5) return;

				// front
				limb[1].setGain(ANGLE, 1.4, 0.02);
				limb[1].setPosition(ANGLE, -S->imu.euler.y - 0.1);
				limb[3].setGain(ANGLE, 1.4, 0.02);
				limb[3].setPosition(ANGLE, -S->imu.euler.y - 0.1);

				P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[1].setGain(EXTENSION, 140, 4);
				limb[1].setPosition(EXTENSION, 0.15);
				limb[3].setGain(EXTENSION, 140, 4);
				limb[3].setPosition(EXTENSION, 0.15);
				// rear
				limb[0].setGain(ANGLE, 1.4, 0.02);
				limb[0].setPosition(ANGLE, -S->imu.euler.y + 0.6); // was 0.5
				limb[2].setGain(ANGLE, 1.4, 0.02);
				limb[2].setPosition(ANGLE, -S->imu.euler.y + 0.6);

				P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[0].setGain(EXTENSION, 180, 5);  // was 120, 4 before
				limb[0].setPosition(EXTENSION, 0.15);
				limb[2].setGain(EXTENSION, 180, 5);  //
				limb[2].setPosition(EXTENSION, 0.15);

				batt_act = S->batt.voltage;
				// batt_act = 16.0;
				for(int i=0; i<8; i++)
				{
					pos_obs[i] = pos_obs_int[i];
					vel_obs[i] = vel_obs_int[i];
				}

				// get new value
				pos_obs_int[0] = toT1(joint[1].getPosition());
				pos_obs_int[1] = toT2(joint[0].getPosition());
				pos_obs_int[4] = toT1(joint[4].getPosition());
				pos_obs_int[5] = toT2(joint[5].getPosition());

				vel_obs_int[0] = joint[1].getVelocity();
				vel_obs_int[1] = -joint[0].getVelocity();
				vel_obs_int[4] = joint[4].getVelocity();
				vel_obs_int[5] = -joint[5].getVelocity();	

				for(int i=0; i<8; i++)
				{
					// pos_act_old[i] = pos_act[i];
					// vel_act_old[i] = vel_act[i];

					pos_act[i] = pos_obs_int[i];
					vel_act[i] = vel_obs_int[i];
				}

				// calculate residual, int has the current value, obs has future value from previous step
				for(int i=0; i<8; i++)
				{
					res_pos[i] = fmodf_mpi_pi(pos_obs[i] - pos_obs_int[i]);
					res_vel[i] = vel_obs[i] - vel_obs_int[i];
				}
				for(int i=0; i<4; i++)
				{
					res_pos_avg[i] = (abs(res_pos[i*2])+abs(res_pos[i*2+1]))*0.5;
					res_vel_avg[i] = (abs(res_vel[i*2])+abs(res_vel[i*2+1]))*0.5;
				}

				// int index = obsCount % 4;
				// if((++obsCount) >= 4) obsCount = 0;
				// pos_obs_seq[index] = pos_obs[0];
				// pos_act_seq[index] = pos_act[0];

				// recov
				if(res_vel_avg[0] > 5 && (res_vel_avg[0] - res_vel_avg[2]) > 5 &&
				(res_pos_avg[0] - res_pos_avg[2]) > 0.03)
				{
					ru_state = 1;
					ru_time = 0;
					mode = FH_RECOV;
					return;
				}

				// step: 0.002s, use the previous PWM
				obs(pos_obs_int, vel_obs_int, batt_act, PWM0, PWM1, 0, 0.002);
				obs(pos_obs_int, vel_obs_int, batt_act, PWM4, PWM5, 2, 0.002);
				// obs_ver(pos_obs_int, vel_obs_int, batt_act, PWM0, PWM1, 0, 0.002, pos_act_old);
				// obs_ver(pos_obs_int, vel_obs_int, batt_act, PWM4, PWM5, 2, 0.002, pos_act_old);

				// back limbs (0 and 2), joint 0/1 and 4/5
				PWM0 = joint[1].getOpenLoop();
				PWM1 = -joint[0].getOpenLoop();
				PWM4 = joint[4].getOpenLoop();
				PWM5 = -joint[5].getOpenLoop();

				// checkPWM[0] = PWM0;
				// checkPWM[1] = PWM1;

				lastTime = S->millis;

				// check
				if(S->millis - startTime > 500)  // TODO: TD detection, 500ms
				{
					ru_state = 5;
					ru_time = 0;
					// t2 = S->millis - startTime;
				}
			}
			else if(ru_state == 5)  // balance
			{
				// ioctl(LOGGER_FILENO, 1); // start logging
				ioctl(LOGGER_FILENO, 0); // do not log
				// front
				limb[1].setGain(ANGLE, 1.2, 0.02);
				limb[1].setPosition(ANGLE, -S->imu.euler.y + 0.1);
				limb[3].setGain(ANGLE, 1.2, 0.02);
				limb[3].setPosition(ANGLE, -S->imu.euler.y + 0.1);

				P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[1].setGain(EXTENSION, 140, 4);
				limb[1].setPosition(EXTENSION, 0.16);  // anchor
				limb[3].setGain(EXTENSION, 140, 4);
				limb[3].setPosition(EXTENSION, 0.16);
				// rear
				limb[0].setGain(ANGLE, 1.2, 0.02);
				limb[0].setPosition(ANGLE, -S->imu.euler.y - 0);
				limb[2].setGain(ANGLE, 1.2, 0.02);
				limb[2].setPosition(ANGLE, -S->imu.euler.y + 0);

				P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[0].setGain(EXTENSION, 140, 4);
				limb[0].setPosition(EXTENSION, 0.16);
				limb[2].setGain(EXTENSION, 140, 4);
				limb[2].setPosition(EXTENSION, 0.16);
				// check
				if((++ru_time) > 200)
				{
					// ru_state = 1;  // reset
					ru_state = 5;  // loop
					ru_time = 0;
					// mode = FH_SW;  // walk forward now
					// mode = FH_STAND;
				}
			}
		}
		else if(mode == FH_RECOV)
		{
			if(recov_start && (++recov_time) < 30)  // record obs a bit, want to get trigger values
			{
				ioctl(LOGGER_FILENO, 1); // log a bit
			}
			else
			{
				ioctl(LOGGER_FILENO, 0); // do not log
				recov_start = false;
				recov_time = 0;
			}

			if(recov_state == 1)  // limb 0 retract, limb 2 extends very fast
			{
				limb[1].setGain(ANGLE, 1.2, 0.02);
				limb[1].setPosition(ANGLE, -S->imu.euler.y + 0.1);
				limb[3].setGain(ANGLE, 1.2, 0.02);
				limb[3].setPosition(ANGLE, -S->imu.euler.y - 0.1);

				P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[1].setGain(EXTENSION, 140, 4);
				limb[1].setPosition(EXTENSION, 0.24);
				limb[3].setGain(EXTENSION, 140, 4);
				limb[3].setPosition(EXTENSION, 0.16);
				// rear
				limb[0].setGain(ANGLE, 1.4, 0.02);
				limb[0].setPosition(ANGLE, -S->imu.euler.y + 0.0);
				limb[2].setGain(ANGLE, 1.4, 0.02);
				limb[2].setPosition(ANGLE, -S->imu.euler.y + 0.3);

				P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[0].setGain(EXTENSION, 150, 5);
				limb[0].setPosition(EXTENSION, 0.10); 
				limb[2].setGain(EXTENSION, 150, 5);
				limb[2].setPosition(EXTENSION, 0.23);

				// check
				if(limb[1].getPosition(EXTENSION) > 0.23)
				{
					recov_state = 2;
				}
			}
			else if(recov_state == 2)  // limb 0 retract
			{
				limb[1].setGain(ANGLE, 1.4, 0.02);
				limb[1].setPosition(ANGLE, -S->imu.euler.y + 0.1);
				limb[3].setGain(ANGLE, 1.4, 0.02);
				limb[3].setPosition(ANGLE, -S->imu.euler.y + 0.1);

				P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[1].setGain(EXTENSION, 120, 4);
				limb[1].setPosition(EXTENSION, 0.16);
				limb[3].setGain(EXTENSION, 120, 4);
				limb[3].setPosition(EXTENSION, 0.16);
				// rear
				limb[0].setGain(ANGLE, 1.4, 0.02);
				limb[0].setPosition(ANGLE, -S->imu.euler.y + 0.6);
				limb[2].setGain(ANGLE, 1.4, 0.02);
				limb[2].setPosition(ANGLE, -S->imu.euler.y + 0.6);

				P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[0].setGain(EXTENSION, 150, 5);
				limb[0].setPosition(EXTENSION, 0.12); // lower retraction account for the obstacle
				limb[2].setGain(EXTENSION, 120, 5);
				limb[2].setPosition(EXTENSION, 0.15);

				// check
				if((++recov_time) > 500)
				{
					recov_state = 2;
					// recov_state = 1;
					recov_time = 0;
					mode = FH_RECOV;
				}
			}
		}
	}

	// add IMU to detect states? two/one stair
	// leg slipping

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
	myData_buf[0] = res_pos_avg[0];
	myData_buf[1] = res_pos_avg[1];
	myData_buf[2] = res_pos_avg[2];
	myData_buf[3] = res_pos_avg[3];
	myData_buf[4] = res_vel_avg[0];
	myData_buf[5] = res_vel_avg[1];
	myData_buf[6] = res_vel_avg[2];
	myData_buf[7] = res_vel_avg[3];
	// myData_buf[7] = nowTime;

	// myData_buf[0] = obsNowTime;
	// myData_buf[1] = obsTimes;
	// myData_buf[2] = vel_obs[0];
	// myData_buf[3] = -vel_obs[1];


	// myData_buf[4] = joint[1].getPosition();
	// myData_buf[5] = joint[0].getPosition();
	// myData_buf[6] = joint[1].getVelocity();
	// myData_buf[7] = joint[0].getVelocity();

	// myData_buf[4] = toQ1(pos_act[0]);
	// myData_buf[5] = toQ2(pos_act[1]);
	// myData_buf[6] = vel_act[0];
	// myData_buf[7] = -vel_act[1];

	// myData_buf[4] = t1;
	// myData_buf[5] = t2;
	// myData_buf[6] = limb[0].getForce(EXTENSION);
	// myData_buf[7] = limb[2].getForce(EXTENSION);

	// myData_buf[0] = pos_obs_seq[0];
	// myData_buf[1] = pos_obs_seq[1];
	// myData_buf[2] = pos_obs_seq[2];
	// myData_buf[3] = pos_obs_seq[3];
	// myData_buf[4] = pos_act_seq[0];
	// myData_buf[5] = pos_act_seq[1];
	// myData_buf[6] = pos_act_seq[2];
	// myData_buf[7] = pos_act_seq[3];

	write(LOGGER_FILENO, myData, 32);
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

		// else if(mode == FH_FU)
		// {
		// 	C->mode = RobotCommand_Mode_LIMB;
		// 	// ioctl(LOGGER_FILENO, 1); // log
		// 	ioctl(LOGGER_FILENO, 0); // do not log

		// 	if(fu_state == 1)  // in starting position
		// 	{
		// 		// ioctl(LOGGER_FILENO, 0); // do not log
		// 		// front
		// 		limb[1].setGain(ANGLE, 1.2, 0.02);
		// 		limb[1].setPosition(ANGLE, -S->imu.euler.y - 0.1);
		// 		limb[3].setGain(ANGLE, 1.2, 0.02);
		// 		limb[3].setPosition(ANGLE, -S->imu.euler.y - 0.1);

		// 		P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
		// 		P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
		// 		limb[1].setGain(EXTENSION, 140, 4);
		// 		limb[1].setPosition(EXTENSION, 0.15);
		// 		limb[3].setGain(EXTENSION, 140, 4);
		// 		limb[3].setPosition(EXTENSION, 0.15);
		// 		// rear
		// 		limb[0].setGain(ANGLE, 1.4, 0.02);
		// 		limb[0].setPosition(ANGLE, -S->imu.euler.y - 0.1);
		// 		limb[2].setGain(ANGLE, 1.4, 0.02);
		// 		limb[2].setPosition(ANGLE, -S->imu.euler.y - 0.1);

		// 		P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
		// 		P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
		// 		limb[0].setGain(EXTENSION, 180, 4);
		// 		limb[0].setPosition(EXTENSION, 0.18);
		// 		limb[2].setGain(EXTENSION, 180, 4);
		// 		limb[2].setPosition(EXTENSION, 0.18);
		// 		// check
		// 		if((++fu_time) > 500)
		// 		{
		// 			fu_state = 2;
		// 			fu_time = 0;
		// 		}
		// 	}
		// 	else if(fu_state == 2)  // front extend, rear hold
		// 	{
		// 		// front
		// 		limb[1].setGain(ANGLE, 1.4, 0.02);
		// 		limb[1].setPosition(ANGLE, -S->imu.euler.y - 0.1);
		// 		limb[3].setGain(ANGLE, 1.4, 0.02);
		// 		limb[3].setPosition(ANGLE, -S->imu.euler.y - 0.1);

		// 		P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
		// 		P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
		// 		float comp = 0.1*(limb[1].getPosition(EXTENSION) - limb[3].getPosition(EXTENSION));
		// 		if(comp <= 0)
		// 		{
		// 			limb[1].setOpenLoop(EXTENSION, fu_pwm);
		// 			limb[3].setOpenLoop(EXTENSION, fu_pwm+comp);
		// 		}
		// 		else{
		// 			limb[1].setOpenLoop(EXTENSION, fu_pwm-comp);  // push ground
		// 			limb[3].setOpenLoop(EXTENSION, fu_pwm);					
		// 		}
		// 		// rear
		// 		limb[0].setGain(ANGLE, 1.4, 0.02);
		// 		limb[0].setPosition(ANGLE, -S->imu.euler.y - 0.1);
		// 		limb[2].setGain(ANGLE, 1.4, 0.02);
		// 		limb[2].setPosition(ANGLE, -S->imu.euler.y - 0.1);

		// 		P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
		// 		P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
		// 		limb[0].setGain(EXTENSION, 180, 4);
		// 		limb[0].setPosition(EXTENSION, 0.20);
		// 		limb[2].setGain(EXTENSION, 180, 4);
		// 		limb[2].setPosition(EXTENSION, 0.20);
		// 		// check
		// 		if(limb[1].getPosition(EXTENSION) > 2.7 &&
		// 			limb[3].getPosition(EXTENSION) > 2.7) fu_state = 3;	
		// 	}
		// 	else if(fu_state == 3)  // front retract/jump, rear hold
		// 	{
		// 		// front
		// 		limb[1].setGain(ANGLE, 1.0, 0.02);
		// 		limb[1].setPosition(ANGLE, -S->imu.euler.y + 0);
		// 		limb[3].setGain(ANGLE, 1.0, 0.02);
		// 		limb[3].setPosition(ANGLE, -S->imu.euler.y + 0);

		// 		P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
		// 		P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
		// 		limb[1].setGain(EXTENSION, 200, 4);
		// 		limb[1].setPosition(EXTENSION, 0.11);
		// 		limb[3].setGain(EXTENSION, 200, 4);
		// 		limb[3].setPosition(EXTENSION, 0.11);
		// 		// rear
		// 		limb[0].setGain(ANGLE, 1.5, 0.03);
		// 		limb[0].setPosition(ANGLE, -S->imu.euler.y + 0.2);  // move rear legs forward?
		// 		limb[2].setGain(ANGLE, 1.5, 0.03);
		// 		limb[2].setPosition(ANGLE, -S->imu.euler.y + 0.2);

		// 		P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
		// 		P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
		// 		limb[0].setGain(EXTENSION, 180, 4);
		// 		limb[0].setPosition(EXTENSION, 0.22);
		// 		limb[2].setGain(EXTENSION, 180, 4);
		// 		limb[2].setPosition(EXTENSION, 0.22);

		// 		// check
		// 		if(limb[1].getPosition(EXTENSION) < 0.115 &&
		// 			limb[3].getPosition(EXTENSION) < 0.115) fu_state = 4;
		// 	}
		// 	else if(fu_state == 4)  // front move forward in air to TD, rear hold
		// 	{
		// 		// front
		// 		limb[1].setGain(ANGLE, 1.4, 0.02);
		// 		limb[1].setPosition(ANGLE, -S->imu.euler.y + 0.9);
		// 		limb[3].setGain(ANGLE, 1.4, 0.02);
		// 		limb[3].setPosition(ANGLE, -S->imu.euler.y + 0.9);

		// 		P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
		// 		P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
		// 		limb[1].setGain(EXTENSION, 120, 4);
		// 		limb[1].setPosition(EXTENSION, 0.17);
		// 		limb[3].setGain(EXTENSION, 120, 4);
		// 		limb[3].setPosition(EXTENSION, 0.17);
		// 		// rear
		// 		limb[0].setGain(ANGLE, 1.5, 0.03);
		// 		limb[0].setPosition(ANGLE, -S->imu.euler.y - 0.3);
		// 		limb[2].setGain(ANGLE, 1.5, 0.03);
		// 		limb[2].setPosition(ANGLE, -S->imu.euler.y - 0.3);

		// 		P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
		// 		P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
		// 		limb[0].setGain(EXTENSION, 180, 4);
		// 		limb[0].setPosition(EXTENSION, 0.24);  // EXTEND MORE TO HIT GROUND
		// 		limb[2].setGain(EXTENSION, 180, 4);
		// 		limb[2].setPosition(EXTENSION, 0.24);
		// 		// check
		// 		if((++fu_time) > 500)  // TODO: TD detection
		// 		{
		// 			fu_state = 5;
		// 			fu_time = 0;
		// 			// mode = FH_STAND;
		// 			// mode = FH_RU;
		// 		}		
		// 	}
		// 	else if(fu_state == 5)  // lift up, balance
		// 	{
		// 		// front
		// 		limb[1].setGain(ANGLE, 0.9, 0.02);
		// 		limb[1].setPosition(ANGLE, -S->imu.euler.y + 0.1);
		// 		limb[3].setGain(ANGLE, 0.9, 0.02);
		// 		limb[3].setPosition(ANGLE, -S->imu.euler.y + 0.1);

		// 		P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
		// 		P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
		// 		limb[1].setGain(EXTENSION, 180, 5);
		// 		limb[1].setPosition(EXTENSION, 0.17);
		// 		limb[3].setGain(EXTENSION, 180, 5);
		// 		limb[3].setPosition(EXTENSION, 0.17);
		// 		// rear
		// 		limb[0].setGain(ANGLE, 1.4, 0.02);
		// 		limb[0].setPosition(ANGLE, -S->imu.euler.y - 0);
		// 		limb[2].setGain(ANGLE, 1.4, 0.02);
		// 		limb[2].setPosition(ANGLE, -S->imu.euler.y - 0);

		// 		P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
		// 		P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
		// 		limb[0].setGain(EXTENSION, 120, 4);
		// 		limb[0].setPosition(EXTENSION, 0.24);
		// 		limb[2].setGain(EXTENSION, 120, 4);
		// 		limb[2].setPosition(EXTENSION, 0.24);
		// 		// check
		// 		if((++fu_time) > 500)
		// 		// if(limb[1].getPosition(ANGLE) < -S->imu.euler.y + 0.25)  // TODO: TD detection
		// 		{
		// 			fu_state = 1;
		// 			fu_time = 0;
		// 			fu_pwm += 0.02;  // compensante for torque loss
		// 			mode = FH_RU;
		// 		}
		// 	}
		// }