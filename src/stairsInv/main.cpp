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
#include <obsValue.h>
#include <observer.h>

#include <string>
#include <iostream>

#if defined(ROBOT_MINITAUR)
// Subject to change for individual robots
const float motZeros[8] = {2.570, 2.036, 3.777, 3.853, 2.183, 1.556, .675, 2.679}; // RML Ellie
// const float motZeros[8] = {0.631, 4.076, 1.852, 3.414, 1.817, 5.500, 1.078, 6.252}; //RML Odie
#endif

struct WalkParam
{
	float apExt;
	float tdExt;
	float angle;
	int   t_side;
	int   t_bottom;
	int   period;
	int   offset;
};

char myData[32];
float *myData_buf = (float*)myData;

// State machine representation of behavior
enum FHMode
{
	FH_SIT = 0,
	FH_STAND,
	FH_FIRST,
	FH_FU,
	FH_RU,
	FH_SW,
	FH_GW
};

float qIn[800];
float qOut[800];
float qInS[600];
float qOutS[600];

float pos_obs_int[8] = {0,0,0,0,0,0,0,0};
float vel_obs_int[8] = {0,0,0,0,0,0,0,0};
float pos_obs[8] = {0,0,0,0,0,0,0,0};
float vel_obs[8] = {0,0,0,0,0,0,0,0};

float res_pos[8] = {0,0,0,0,0,0,0,0};
float res_vel[8] = {0,0,0,0,0,0,0,0};
float res_pos_avg[4] = {0,0,0,0};  // four limbs
float res_vel_avg[4] = {0,0,0,0};

// recovery

class Stairs : public ReorientableBehavior
{
public:
	FHMode mode = FH_SIT; // Current state within state-machine

	int t;
	int t_left = 0;
	int t_right = 0;
	float step = 0.001;

	// jump
	int ru_time = 0;
	int ru_state = 1;
	int fu_time = 0;
	int fu_state = 1;
	float fu_pwm = 0.85;  // start with 0.5 PWM
	int ru_count = 0;

	// walk
	bool gwStart = true;
	bool swStart = true;
	int gwStartTimeResidual = 0;
	int swStartTimeResidual = 0;
	int gwStartTime;

	int walk_time = 0;
	bool ifFirstStair = true;
	bool wait = false;
	int waitTime = 0;
	int startTime = 0;

	// apExt, tdExt, angle, t_side, t_bottom_ period, offset
	WalkParam gW = {0.12, 0.18, -15, 100, 300, 800, 400};;
	WalkParam sW = {0.13, 0.18, -5,  100, 200, 600, 300};;

	float integral[8] = {0,0,0,0,0,0,0,0};
	float pwm[8];
	float kpl = 0.3;
	float kpm = 0.6;
	float kph = 0.9;
	float kd = 0.01;
	float ki = 0.1;

	// observer
	float batt_act;

	// use joystick to start walking or stop
	void signal(uint32_t sig)
	{
		if (sig == 3)
		{
			mode = FH_GW;
		}
		else
		{
			mode = FH_STAND;
		}
	}
	void begin()
	{
		mode = FH_STAND;			// Start behavior in STAND mode

		getTrajInterp(qIn, qOut, gW.apExt, gW.tdExt, gW.angle, gW.t_bottom, gW.t_side);  // get traj on ground

	 	getTrajRotInterp(qInS, qOutS, sW.apExt, sW.tdExt, sW.angle, sW.t_bottom, sW.t_side);  // get traj on stair before jumping up, reuse same array

		for(int i=0; i<4; i++) P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;
	}

	// STAND, GW, FIRST, SW, FU, RU, SW, FU, RU...
	void update()
	{
		// ioctl(LOGGER_FILENO, 1); // start logging
		// C->mode = RobotCommand_Mode_JOINT;

		if (mode == FH_STAND)
		{
			C->mode = RobotCommand_Mode_JOINT;  // !!!!!!!
			ioctl(LOGGER_FILENO, 0); // do not log

			for(int i=0; i<8; i++)
			{
				if( i==1 || i==6 || i==4 || i==3 )  // in
				{
					integral[i] += step*(qIn[0]-joint[i].getPosition());
				  pwm[i] = kpm*(qIn[0]-joint[i].getPosition())+kd*(0-joint[i].getVelocity()+ki*integral[i]);
				}
				else if( i==0 || i==7 || i==5 || i==2 )  // out
				{
				  integral[i] += step*(qOut[0]-joint[i].getPosition());
					pwm[i] = kpm*(qOut[0]-joint[i].getPosition())+kd*(0-joint[i].getVelocity()+ki*integral[i]);
				}
			}
			for(int i=0; i<8; i++)
			{
				joint[i].setOpenLoop(constrain(pwm[i],-1,1));
			}
		}
		else if (mode == FH_GW)  // ground walking
		{
			C->mode = RobotCommand_Mode_JOINT;
			ioctl(LOGGER_FILENO, 0); // do not log
			// ioctl(LOGGER_FILENO, 1); // log

			if(gwStart)
			{
				gwStartTime = S->millis;
				gwStartTimeResidual = S->millis % gW.period;
				gwStart = false;
			}
			
			t_left = (S->millis - gwStartTimeResidual) % gW.period;
			t_right = (S->millis - gwStartTimeResidual + gW.offset) % gW.period;

			for(int i=0; i<8; i++)
			{
				if( i==1 || i==6 )  // left in
				{
					pwm[i] = kph*(qIn[t_left]-joint[i].getPosition())+kd*(0-joint[i].getVelocity());
				}
				else if( i==0 || i==7 )  // left out
				{
					pwm[i] = kph*(qOut[t_left]-joint[i].getPosition())+kd*(0-joint[i].getVelocity());
				}
				else if( i==4 || i==3 )  // right in
				{
					pwm[i] = kph*(qIn[t_right]-joint[i].getPosition())+kd*(0-joint[i].getVelocity());
				}
				else if( i==5 || i==2 ) // right out
				{
					pwm[i] = kph*(qOut[t_right]-joint[i].getPosition())+kd*(0-joint[i].getVelocity());
				}
			}

			for(int i=0; i<8; i++)
			{
				joint[i].setOpenLoop(constrain(pwm[i],-1,1));
			}

			if((++startTime) < 100) return;  // ignore first 100ms for detection

			res_pos[6] = abs(joint[6].getPosition() - q1_obs[t_left - 350]);
			res_pos[7] = abs(joint[7].getPosition() - q2_obs[t_left - 350]);
			res_pos_avg[3] = (res_pos[6]+res_pos[7])*0.5;
			res_pos[3] = abs(joint[3].getPosition() - q1_obs[t_right - 350]);
			res_pos[2] = abs(joint[2].getPosition() - q2_obs[t_right - 350]);
			res_pos_avg[1] = (res_pos[3]+res_pos[2])*0.5;

			if((S->millis - gwStartTime) > 2000)
			// if(wait)
			{
				if((++waitTime) > 30) mode = FH_FIRST;
				return;
			}

			// check hit
			// if(t_left >= 400 && t_left <= 470 && res_pos_avg[3] > 0.05) wait = true;
			// if(t_right >= 400 && t_right <= 465 && res_pos_avg[1] > 0.04) wait = true;	
		}
		else if (mode == FH_SW)  // stair walking
		{
			C->mode = RobotCommand_Mode_JOINT;
			ioctl(LOGGER_FILENO, 0); // do not log

			if(ru_count == 4)
			{
				mode = FH_GW;
				return;
			}

			if(swStart)
			{
				swStartTimeResidual = S->millis % sW.period;
				swStart = false;
			}
			
			t_left = (S->millis - swStartTimeResidual + 600) % sW.period;
			t_right = (S->millis - swStartTimeResidual + sW.offset + 600) % sW.period;

			for(int i=0; i<8; i++)
			{
				if( i==1 || i==6 )  // left in
				{
					pwm[i] = kph*(qInS[t_left]-joint[i].getPosition())+kd*(0-joint[i].getVelocity());
				}
				else if( i==0 || i==7 )  // left out
				{
					pwm[i] = kph*(qOutS[t_left]-joint[i].getPosition())+kd*(0-joint[i].getVelocity());
				}
				else if( i==4 || i==3 )  // right in
				{
					pwm[i] = kph*(qInS[t_right]-joint[i].getPosition())+kd*(0-joint[i].getVelocity());
				}
				else if( i==5 || i==2 ) // right out
				{
					pwm[i] = kph*(qOutS[t_right]-joint[i].getPosition())+kd*(0-joint[i].getVelocity());
				}
			}

			for(int i=0; i<8; i++)
			{
				joint[i].setOpenLoop(constrain(pwm[i],-1,1));
			}

			float thres = ifFirstStair ? 3600 : 900;

			if((++walk_time) > thres)  // 
			{
				walk_time = 0;
				ifFirstStair = false;
				mode = FH_FU;  // bigger front jump now
			}
		}
		else if (mode == FH_FIRST)
		{
			C->mode = RobotCommand_Mode_LIMB;
			ioctl(LOGGER_FILENO, 0); // do not log

			gwStart = true;
			swStart = true;

			if(fu_state == 1)  // in starting position
			{
				// front
				limb[1].setGain(ANGLE, 0.9, 0.02);
				limb[1].setPosition(ANGLE, -S->imu.euler.y + 0);
				limb[3].setGain(ANGLE, 0.9, 0.02);
				limb[3].setPosition(ANGLE, -S->imu.euler.y + 0);

				P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[1].setGain(EXTENSION, 120, 4);
				limb[1].setPosition(EXTENSION, 0.18);
				limb[3].setGain(EXTENSION, 120, 4);
				limb[3].setPosition(EXTENSION, 0.18);
				// rear
				limb[0].setGain(ANGLE, 0.9, 0.02);
				limb[0].setPosition(ANGLE, -S->imu.euler.y - 0.1);
				limb[2].setGain(ANGLE, 0.9, 0.02);
				limb[2].setPosition(ANGLE, -S->imu.euler.y - 0.1);

				P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[0].setGain(EXTENSION, 120, 4);
				limb[0].setPosition(EXTENSION, 0.18);
				limb[2].setGain(EXTENSION, 120, 4);
				limb[2].setPosition(EXTENSION, 0.18);
				// check
				if((++fu_time) > 1000)  // long delay before jump
				{
					fu_state = 2;
					// fu_state = 1;  // loop here
					fu_time = 0;
				}
			}
			else if(fu_state == 2)  // front extend, rear hold
			{
				// front
				limb[1].setGain(ANGLE, 1.4, 0.02);
				limb[1].setPosition(ANGLE, -S->imu.euler.y + 0);
				limb[3].setGain(ANGLE, 1.4, 0.02);
				limb[3].setPosition(ANGLE, -S->imu.euler.y + 0);

				P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
				P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
				float comp = 0;
				// float comp = 0.1*(limb[1].getPosition(EXTENSION) - limb[3].getPosition(EXTENSION));
				if(comp <= 0)
				{
					limb[1].setOpenLoop(EXTENSION, 1.05);
					limb[3].setOpenLoop(EXTENSION, 1.05+comp);
				}
				else{
					limb[1].setOpenLoop(EXTENSION, 1.05-comp);  // push ground
					limb[3].setOpenLoop(EXTENSION, 1.05);					
				}
				// rear
				limb[0].setGain(ANGLE, 1.4, 0.02);
				limb[0].setPosition(ANGLE, -S->imu.euler.y - 0.1);
				limb[2].setGain(ANGLE, 1.4, 0.02);
				limb[2].setPosition(ANGLE, -S->imu.euler.y - 0.1);

				P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[0].setGain(EXTENSION, 180, 4);
				limb[0].setPosition(EXTENSION, 0.18);
				limb[2].setGain(EXTENSION, 180, 4);
				limb[2].setPosition(EXTENSION, 0.18);
				// check
				if(limb[1].getPosition(EXTENSION) > 2.7 &&
					limb[3].getPosition(EXTENSION) > 2.7) fu_state = 3;	
			}
			else if(fu_state == 3)  // front retract/jump, rear hold
			{
				// front
				limb[1].setGain(ANGLE, 1.4, 0.02);
				limb[1].setPosition(ANGLE, -S->imu.euler.y + 0);
				limb[3].setGain(ANGLE, 1.4, 0.02);
				limb[3].setPosition(ANGLE, -S->imu.euler.y + 0);

				P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[1].setGain(EXTENSION, 200, 4);
				limb[1].setPosition(EXTENSION, 0.11);
				limb[3].setGain(EXTENSION, 200, 4);
				limb[3].setPosition(EXTENSION, 0.11);
				// rear
				limb[0].setGain(ANGLE, 1.4, 0.02);
				limb[0].setPosition(ANGLE, -S->imu.euler.y - 0.1);
				limb[2].setGain(ANGLE, 1.4, 0.02);
				limb[2].setPosition(ANGLE, -S->imu.euler.y - 0.1);

				P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[0].setGain(EXTENSION, 180, 4);
				limb[0].setPosition(EXTENSION, 0.18);
				limb[2].setGain(EXTENSION, 180, 4);
				limb[2].setPosition(EXTENSION, 0.18);
				// check
				if(limb[1].getPosition(EXTENSION) < 0.115 &&
					limb[3].getPosition(EXTENSION) < 0.115) fu_state = 4;
			}
			else if(fu_state == 4)  // front move forward in air to TD, rear hold
			{
				// front
				limb[1].setGain(ANGLE, 1.2, 0.02);
				limb[1].setPosition(ANGLE, -S->imu.euler.y + 0.7);  // was 0.6 before
				limb[3].setGain(ANGLE, 1.2, 0.02);
				limb[3].setPosition(ANGLE, -S->imu.euler.y + 0.7);

				P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[1].setGain(EXTENSION, 120, 4);
				limb[1].setPosition(EXTENSION, 0.15);
				limb[3].setGain(EXTENSION, 120, 4);
				limb[3].setPosition(EXTENSION, 0.15);
				// rear
				limb[0].setGain(ANGLE, 1.4, 0.02);
				limb[0].setPosition(ANGLE, -S->imu.euler.y - 0.3);
				limb[2].setGain(ANGLE, 1.4, 0.02);
				limb[2].setPosition(ANGLE, -S->imu.euler.y - 0.3);

				P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[0].setGain(EXTENSION, 180, 4);
				limb[0].setPosition(EXTENSION, 0.18);
				limb[2].setGain(EXTENSION, 180, 4);
				limb[2].setPosition(EXTENSION, 0.18);
				// check
				if((++fu_time) > 500)
				{
					fu_state = 5;
					fu_time = 0;
				}
			}
			else if(fu_state == 5)  // balance
			{
				// front
				limb[1].setGain(ANGLE, 1.2, 0.02);
				limb[1].setPosition(ANGLE, -S->imu.euler.y + 0.1);
				limb[3].setGain(ANGLE, 1.2, 0.02);
				limb[3].setPosition(ANGLE, -S->imu.euler.y + 0.1);

				P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[1].setGain(EXTENSION, 140, 4);
				limb[1].setPosition(EXTENSION, 0.15);
				limb[3].setGain(EXTENSION, 140, 4);
				limb[3].setPosition(EXTENSION, 0.15);
				// rear
				limb[0].setGain(ANGLE, 1.4, 0.02);
				limb[0].setPosition(ANGLE, -S->imu.euler.y - 0.1);
				limb[2].setGain(ANGLE, 1.4, 0.02);
				limb[2].setPosition(ANGLE, -S->imu.euler.y - 0.1);

				P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[0].setGain(EXTENSION, 180, 4);
				limb[0].setPosition(EXTENSION, 0.18);
				limb[2].setGain(EXTENSION, 180, 4);
				limb[2].setPosition(EXTENSION, 0.18);
				// check
				if((++fu_time) > 200)
				{
					fu_state = 1;
					fu_time = 0;
					mode = FH_SW;
				}
			}
		}
		else if(mode == FH_FU)
		{
			for(int i=0; i<4; i++)
			{
				res_pos_avg[i] = 0;
				res_vel_avg[i] = 0;
 			}

			C->mode = RobotCommand_Mode_LIMB;
			ioctl(LOGGER_FILENO, 0); // do not log

			gwStart = true;
			swStart = true;

			if(fu_state == 1)  // in starting position
			{
				// front
				limb[1].setGain(ANGLE, 1.2, 0.02);
				limb[1].setPosition(ANGLE, -S->imu.euler.y - 0.1);
				limb[3].setGain(ANGLE, 1.2, 0.02);
				limb[3].setPosition(ANGLE, -S->imu.euler.y - 0.1);

				P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[1].setGain(EXTENSION, 140, 4);
				limb[1].setPosition(EXTENSION, 0.15);
				limb[3].setGain(EXTENSION, 140, 4);
				limb[3].setPosition(EXTENSION, 0.15);
				// rear
				limb[0].setGain(ANGLE, 1.4, 0.02);
				limb[0].setPosition(ANGLE, -S->imu.euler.y - 0.1);
				limb[2].setGain(ANGLE, 1.4, 0.02);
				limb[2].setPosition(ANGLE, -S->imu.euler.y - 0.1);

				P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[0].setGain(EXTENSION, 180, 4);
				limb[0].setPosition(EXTENSION, 0.18);
				limb[2].setGain(EXTENSION, 180, 4);
				limb[2].setPosition(EXTENSION, 0.18);
				// check
				if((++fu_time) > 500)
				{
					fu_state = 2;
					fu_time = 0;
				}
			}
			else if(fu_state == 2)  // front extend, rear hold
			{
				// front
				limb[1].setGain(ANGLE, 1.4, 0.02);
				limb[1].setPosition(ANGLE, -S->imu.euler.y - 0.1);
				limb[3].setGain(ANGLE, 1.4, 0.02);
				limb[3].setPosition(ANGLE, -S->imu.euler.y - 0.1);

				P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
				P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
				float comp = 0;
				// float comp = 0.1*(limb[1].getPosition(EXTENSION) - limb[3].getPosition(EXTENSION));
				if(comp <= 0)
				{
					limb[1].setOpenLoop(EXTENSION, fu_pwm);
					limb[3].setOpenLoop(EXTENSION, fu_pwm+comp);
				}
				else{
					limb[1].setOpenLoop(EXTENSION, fu_pwm-comp);  // push ground
					limb[3].setOpenLoop(EXTENSION, fu_pwm);					
				}
				// rear
				limb[0].setGain(ANGLE, 1.4, 0.02);
				limb[0].setPosition(ANGLE, -S->imu.euler.y - 0.1);
				limb[2].setGain(ANGLE, 1.4, 0.02);
				limb[2].setPosition(ANGLE, -S->imu.euler.y - 0.1);

				P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[0].setGain(EXTENSION, 180, 4);
				limb[0].setPosition(EXTENSION, 0.20);
				limb[2].setGain(EXTENSION, 180, 4);
				limb[2].setPosition(EXTENSION, 0.20);
				// check
				if(limb[1].getPosition(EXTENSION) > 2.7 &&
					limb[3].getPosition(EXTENSION) > 2.7) fu_state = 3;	
			}
			else if(fu_state == 3)  // front retract/jump, rear hold
			{
				ioctl(LOGGER_FILENO, 1); // start logging

				// front
				limb[1].setGain(ANGLE, 1.0, 0.02);
				limb[1].setPosition(ANGLE, -S->imu.euler.y + 0);
				limb[3].setGain(ANGLE, 1.0, 0.02);
				limb[3].setPosition(ANGLE, -S->imu.euler.y + 0);

				P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[1].setGain(EXTENSION, 200, 4);
				limb[1].setPosition(EXTENSION, 0.11);
				limb[3].setGain(EXTENSION, 200, 4);
				limb[3].setPosition(EXTENSION, 0.11);
				// rear
				limb[0].setGain(ANGLE, 1.5, 0.03);
				limb[0].setPosition(ANGLE, -S->imu.euler.y + 0.2);  // move rear legs forward?
				limb[2].setGain(ANGLE, 1.5, 0.03);
				limb[2].setPosition(ANGLE, -S->imu.euler.y + 0.2);

				P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[0].setGain(EXTENSION, 180, 4);
				limb[0].setPosition(EXTENSION, 0.22);
				limb[2].setGain(EXTENSION, 180, 4);
				limb[2].setPosition(EXTENSION, 0.22);

				// get new value
				batt_act = S->batt.voltage;
				for(int i=0; i<8; i++)
				{
					pos_obs[i] = pos_obs_int[i];
					vel_obs[i] = vel_obs_int[i];
				}

				pos_obs_int[2] = toT1(joint[3].getPosition());
				pos_obs_int[3] = toT2(joint[2].getPosition());
				pos_obs_int[6] = toT1(joint[6].getPosition());
				pos_obs_int[7] = toT2(joint[7].getPosition());

				vel_obs_int[2] = joint[3].getVelocity();
				vel_obs_int[3] = -joint[2].getVelocity();	
				vel_obs_int[6] = joint[6].getVelocity();
				vel_obs_int[7] = -joint[7].getVelocity();	

				// calculate residual, int has the current value, obs has future value from previous step
				for(int i=0; i<8; i++)
				{
					res_pos[i] = pos_obs[i] - pos_obs_int[i];
					res_vel[i] = vel_obs[i] - vel_obs_int[i];
				}
				// for(int i=0; i<4; i++)
				// {
				// 	res_pos_avg[i] = (abs(res_pos[i*2])+abs(res_pos[i*2+1]))*0.5;
				// 	res_vel_avg[i] = (abs(res_vel[i*2])+abs(res_vel[i*2+1]))*0.5;
				// }
				res_pos_avg[1] = (abs(res_pos[2])+abs(res_pos[3]))*0.5;
				res_pos_avg[3] = (abs(res_pos[6])+abs(res_pos[7]))*0.5;
				res_vel_avg[1] = (abs(res_vel[2])+abs(res_vel[3]))*0.5;
				res_vel_avg[3] = (abs(res_vel[6])+abs(res_vel[7]))*0.5;

				// front limbs (1 and 3), joint 3/2 and 6/7
				float PWM2 = joint[2].getOpenLoop();
				float PWM3 = joint[3].getOpenLoop();
				float PWM6 = joint[6].getOpenLoop();
				float PWM7 = joint[7].getOpenLoop();

				// step: 0.002s
				obs(pos_obs_int, vel_obs_int, batt_act, PWM3, PWM2, 1, 0.002);
				obs(pos_obs_int, vel_obs_int, batt_act, PWM6, PWM7, 3, 0.002);

				// check
				if(limb[1].getPosition(EXTENSION) < 0.115 &&
					limb[3].getPosition(EXTENSION) < 0.115) fu_state = 4;
			}
			else if(fu_state == 4)  // front move forward in air to TD, rear hold
			{
				ioctl(LOGGER_FILENO, 1); // start logging

				// front
				limb[1].setGain(ANGLE, 1.4, 0.02);
				limb[1].setPosition(ANGLE, -S->imu.euler.y + 0.9);
				limb[3].setGain(ANGLE, 1.4, 0.02);
				limb[3].setPosition(ANGLE, -S->imu.euler.y + 0.9);

				P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[1].setGain(EXTENSION, 120, 4);
				limb[1].setPosition(EXTENSION, 0.17);
				limb[3].setGain(EXTENSION, 120, 4);
				limb[3].setPosition(EXTENSION, 0.17);
				// rear
				limb[0].setGain(ANGLE, 1.5, 0.03);
				limb[0].setPosition(ANGLE, -S->imu.euler.y - 0.3);
				limb[2].setGain(ANGLE, 1.5, 0.03);
				limb[2].setPosition(ANGLE, -S->imu.euler.y - 0.3);

				P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[0].setGain(EXTENSION, 180, 4);
				limb[0].setPosition(EXTENSION, 0.24);  // EXTEND MORE TO HIT GROUND
				limb[2].setGain(EXTENSION, 180, 4);
				limb[2].setPosition(EXTENSION, 0.24);

				// get new value
				batt_act = S->batt.voltage;
				for(int i=0; i<8; i++)
				{
					pos_obs[i] = pos_obs_int[i];
					vel_obs[i] = vel_obs_int[i];
				}

				pos_obs_int[0] = toT1(joint[1].getPosition());
				pos_obs_int[1] = toT2(joint[0].getPosition());
				pos_obs_int[4] = toT1(joint[4].getPosition());
				pos_obs_int[5] = toT2(joint[5].getPosition());

				vel_obs_int[0] = joint[1].getVelocity();
				vel_obs_int[1] = -joint[0].getVelocity();
				vel_obs_int[4] = joint[4].getVelocity();
				vel_obs_int[5] = -joint[5].getVelocity();	

				// calculate residual, int has the current value, obs has future value from previous step
				for(int i=0; i<8; i++)
				{
					res_pos[i] = pos_obs[i] - pos_obs_int[i];
					res_vel[i] = vel_obs[i] - vel_obs_int[i];
				}
				// for(int i=0; i<4; i++)
				// {
				// 	res_pos_avg[i] = (abs(res_pos[i*2])+abs(res_pos[i*2+1]))*0.5;
				// 	res_vel_avg[i] = (abs(res_vel[i*2])+abs(res_vel[i*2+1]))*0.5;
				// }

				res_pos_avg[1] = (abs(res_pos[2])+abs(res_pos[3]))*0.5;
				res_pos_avg[3] = (abs(res_pos[6])+abs(res_pos[7]))*0.5;
				res_vel_avg[1] = (abs(res_vel[2])+abs(res_vel[3]))*0.5;
				res_vel_avg[3] = (abs(res_vel[6])+abs(res_vel[7]))*0.5;

				// front limbs (1 and 3), joint 3/2 and 6/7
				float PWM2 = joint[2].getOpenLoop();
				float PWM3 = joint[3].getOpenLoop();
				float PWM6 = joint[6].getOpenLoop();
				float PWM7 = joint[7].getOpenLoop();

				// step: 0.002s
				obs(pos_obs_int, vel_obs_int, batt_act, PWM3, PWM2, 1, 0.002);
				obs(pos_obs_int, vel_obs_int, batt_act, PWM6, PWM7, 3, 0.002);

				// check
				if((++fu_time) > 500)  // TODO: TD detection
				{
					fu_state = 5;
					fu_time = 0;
					// mode = FH_STAND;
					// mode = FH_RU;
				}		
			}
			else if(fu_state == 5)  // lift up, balance
			{
				// front
				limb[1].setGain(ANGLE, 0.9, 0.02);
				limb[1].setPosition(ANGLE, -S->imu.euler.y + 0.1);
				limb[3].setGain(ANGLE, 0.9, 0.02);
				limb[3].setPosition(ANGLE, -S->imu.euler.y + 0.1);

				P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[1].setGain(EXTENSION, 180, 5);
				limb[1].setPosition(EXTENSION, 0.17);
				limb[3].setGain(EXTENSION, 180, 5);
				limb[3].setPosition(EXTENSION, 0.17);
				// rear
				limb[0].setGain(ANGLE, 1.4, 0.02);
				limb[0].setPosition(ANGLE, -S->imu.euler.y - 0);
				limb[2].setGain(ANGLE, 1.4, 0.02);
				limb[2].setPosition(ANGLE, -S->imu.euler.y - 0);

				P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[0].setGain(EXTENSION, 120, 4);
				limb[0].setPosition(EXTENSION, 0.24);
				limb[2].setGain(EXTENSION, 120, 4);
				limb[2].setPosition(EXTENSION, 0.24);
				// check
				if((++fu_time) > 500)
				// if(limb[1].getPosition(ANGLE) < -S->imu.euler.y + 0.25)  // TODO: TD detection
				{
					fu_state = 1;
					fu_time = 0;
					fu_pwm += 0.04;  // compensante for torque loss
					mode = FH_RU;
				}
			}
		}
		else if (mode == FH_RU)
		{
			for(int i=0; i<4; i++)
			{
				res_pos_avg[i] = 0;
				res_vel_avg[i] = 0;
 			}

			C->mode = RobotCommand_Mode_LIMB;
			ioctl(LOGGER_FILENO, 0); // do not log

			gwStart = true;
			swStart = true;

			if(ru_state == 1)  // front hold, rear retract for jump
			{
				// ioctl(LOGGER_FILENO, 1); // start logging
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
				limb[0].setPosition(EXTENSION, 0.19);
				limb[2].setGain(EXTENSION, 100, 3); 
				limb[2].setPosition(EXTENSION, 0.19);
				// check
				if(limb[0].getPosition(EXTENSION) < 0.20 &&
					limb[2].getPosition(EXTENSION) < 0.20) ru_state = 2;
			}
			else if(ru_state == 2)  // front hold, rear extend
			{
				// ioctl(LOGGER_FILENO, 1); // start logging
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
				float comp = 0;
				// float comp = 0.1*(limb[0].getPosition(EXTENSION) - limb[2].getPosition(EXTENSION));
				if(comp <= 0)
				{
					limb[0].setOpenLoop(EXTENSION, 1.2);
					limb[2].setOpenLoop(EXTENSION, 1.2+comp);
				}
				else
				{
					limb[0].setOpenLoop(EXTENSION, 1.2-comp);  // push ground
					limb[2].setOpenLoop(EXTENSION, 1.2);					
				}
				// check
				if(limb[0].getPosition(EXTENSION) > 2.7 &&
					limb[2].getPosition(EXTENSION) > 2.7) ru_state = 3;
			}
			else if(ru_state == 3)  // front hold, rear retract/jump
			{
				ioctl(LOGGER_FILENO, 1); // start logging
				if((++ru_time) < 50)
				{
					limb[1].setGain(ANGLE, 1.6, 0.03);
					limb[1].setPosition(ANGLE, -S->imu.euler.y + 0.30);
					limb[3].setGain(ANGLE, 1.6, 0.03);
					limb[3].setPosition(ANGLE, -S->imu.euler.y + 0.30);

					P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
					P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
					limb[1].setOpenLoop(EXTENSION, 0.05);
					limb[3].setOpenLoop(EXTENSION, 0.05);
				}
				else  // rock forward
				{
					limb[1].setGain(ANGLE, 1.8, 0.03);
					limb[1].setPosition(ANGLE, -S->imu.euler.y - 0.1);
					limb[3].setGain(ANGLE, 1.8, 0.03);
					limb[3].setPosition(ANGLE, -S->imu.euler.y - 0.1);

					P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
					P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
					limb[1].setOpenLoop(EXTENSION, 0.20);
					limb[3].setOpenLoop(EXTENSION, 0.20);
				}
				// rear
				limb[0].setGain(ANGLE, 1.4, 0.02);
				limb[0].setPosition(ANGLE, -S->imu.euler.y - 0.30);
				limb[2].setGain(ANGLE, 1.4, 0.02);
				limb[2].setPosition(ANGLE, -S->imu.euler.y - 0.30);

				P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[0].setGain(EXTENSION, 180, 5);
				limb[0].setPosition(EXTENSION, 0.11);
				limb[2].setGain(EXTENSION, 180, 5);
				limb[2].setPosition(EXTENSION, 0.11);

				// get new value
				batt_act = S->batt.voltage;
				for(int i=0; i<8; i++)
				{
					pos_obs[i] = pos_obs_int[i];
					vel_obs[i] = vel_obs_int[i];
				}

				pos_obs_int[0] = toT1(joint[1].getPosition());
				pos_obs_int[1] = toT2(joint[0].getPosition());
				pos_obs_int[4] = toT1(joint[4].getPosition());
				pos_obs_int[5] = toT2(joint[5].getPosition());

				vel_obs_int[0] = joint[1].getVelocity();
				vel_obs_int[1] = -joint[0].getVelocity();
				vel_obs_int[4] = joint[4].getVelocity();
				vel_obs_int[5] = -joint[5].getVelocity();	

				// calculate residual, int has the current value, obs has future value from previous step
				for(int i=0; i<8; i++)
				{
					res_pos[i] = pos_obs[i] - pos_obs_int[i];
					res_vel[i] = vel_obs[i] - vel_obs_int[i];
				}

				res_pos_avg[0] = (abs(res_pos[0])+abs(res_pos[1]))*0.5;
				res_pos_avg[2] = (abs(res_pos[4])+abs(res_pos[5]))*0.5;
				res_vel_avg[0] = (abs(res_vel[0])+abs(res_vel[1]))*0.5;
				res_vel_avg[2] = (abs(res_vel[4])+abs(res_vel[5]))*0.5;
				
				// back limbs (0 and 2), joint 0/1 and 4/5
				float PWM0 = joint[0].getOpenLoop();
				float PWM1 = joint[1].getOpenLoop();
				float PWM4 = joint[4].getOpenLoop();
				float PWM5 = joint[5].getOpenLoop();

				// step: 0.002s
				obs(pos_obs_int, vel_obs_int, batt_act, PWM1, PWM0, 0, 0.002);
				obs(pos_obs_int, vel_obs_int, batt_act, PWM4, PWM5, 2, 0.002);

				// check
				if(limb[0].getPosition(EXTENSION) < 0.115 &&
					limb[2].getPosition(EXTENSION) < 0.115) 
				{
					ru_state = 4;
					ru_time = 0;
				}
			}
			else if(ru_state == 4)  // front move forward, rear move forward in air to TD
			{
				ioctl(LOGGER_FILENO, 1); // start logging
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
				limb[0].setGain(ANGLE, 1.6, 0.02);
				limb[0].setPosition(ANGLE, -S->imu.euler.y + 0.5);
				limb[2].setGain(ANGLE, 1.6, 0.02);
				limb[2].setPosition(ANGLE, -S->imu.euler.y + 0.5);

				P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[2].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[0].setGain(EXTENSION, 120, 5);
				limb[0].setPosition(EXTENSION, 0.15);
				limb[2].setGain(EXTENSION, 120, 5);
				limb[2].setPosition(EXTENSION, 0.15);

				// get new value
				batt_act = S->batt.voltage;
				for(int i=0; i<8; i++)
				{
					pos_obs[i] = pos_obs_int[i];
					vel_obs[i] = vel_obs_int[i];
				}

				pos_obs_int[0] = toT1(joint[1].getPosition());
				pos_obs_int[1] = toT2(joint[0].getPosition());
				pos_obs_int[4] = toT1(joint[4].getPosition());
				pos_obs_int[5] = toT2(joint[5].getPosition());

				vel_obs_int[0] = joint[1].getVelocity();
				vel_obs_int[1] = -joint[0].getVelocity();
				vel_obs_int[4] = joint[4].getVelocity();
				vel_obs_int[5] = -joint[5].getVelocity();

				// calculate residual, int has the current value, obs has future value from previous step
				for(int i=0; i<8; i++)
				{
					res_pos[i] = pos_obs[i] - pos_obs_int[i];
					res_vel[i] = vel_obs[i] - vel_obs_int[i];
				}

				res_pos_avg[0] = (abs(res_pos[0])+abs(res_pos[1]))*0.5;
				res_pos_avg[2] = (abs(res_pos[4])+abs(res_pos[5]))*0.5;
				res_vel_avg[0] = (abs(res_vel[0])+abs(res_vel[1]))*0.5;
				res_vel_avg[2] = (abs(res_vel[4])+abs(res_vel[5]))*0.5;

				// back limbs (0 and 2), joint 0/1 and 4/5
				float PWM0 = joint[0].getOpenLoop();
				float PWM1 = joint[1].getOpenLoop();
				float PWM4 = joint[4].getOpenLoop();
				float PWM5 = joint[5].getOpenLoop();

				// step: 0.002s
				obs(pos_obs_int, vel_obs_int, batt_act, PWM1, PWM0, 0, 0.002);
				obs(pos_obs_int, vel_obs_int, batt_act, PWM4, PWM5, 2, 0.002);

				// check
				if((++ru_time) > 200)  // TODO: TD detection
				{
					ru_state = 5;
					ru_time = 0;
				}
			}
			else if(ru_state == 5)  // balance
			{
				// ioctl(LOGGER_FILENO, 1); // start logging
				// front
				limb[1].setGain(ANGLE, 1.2, 0.02);
				limb[1].setPosition(ANGLE, -S->imu.euler.y - 0);
				limb[3].setGain(ANGLE, 1.2, 0.02);
				limb[3].setPosition(ANGLE, -S->imu.euler.y - 0);

				P->limbs[1].type = LimbParams_Type_SYMM5BAR_EXT_M;
				P->limbs[3].type = LimbParams_Type_SYMM5BAR_EXT_M;
				limb[1].setGain(EXTENSION, 160, 5);
				limb[1].setPosition(EXTENSION, 0.16);  // anchor
				limb[3].setGain(EXTENSION, 160, 5);
				limb[3].setPosition(EXTENSION, 0.16);
				// rear
				limb[0].setGain(ANGLE, 1.2, 0.02);
				limb[0].setPosition(ANGLE, -S->imu.euler.y + 0);
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
					ru_state = 1;  // reset
					ru_time = 0;
					ru_count++;
					mode = FH_SW;  // walk forward now
					// mode = FH_FU;
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
