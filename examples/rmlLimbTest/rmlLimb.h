#ifndef RMLLIMB_H
#define RMLLIMB_H

// #include <stdio.h>
// #include <SDK.h>
// #include <math.h>
// #include <Motor.h>

/*
TODO:

Fix IK to handle angle wrapping (like FK)
Decide on public/private member variables

*/

class rmlLimb
{
public:
	float l0, l1; 
	int idx0, idx1; 
	int usePolar = 0;
	float q0, q1, dq0, dq1; 
	float kp = 0, kd = 0, kpr = 0, kdr = 0, kpt = 0, kdt = 0; // feedback gains 
	float ur, uth; // [pwm0;pwm1] = J'*[ur;uth] we are going to apply voltage to two motors at the end 

	rmlLimb(); 
	void Init(int legnumber); 

	void updateState(void);
	void updateCommand(void);

	float getPosition(int getAng);
	float getVelocity(int getAng);

	void getJacobian(float q0, float q1, float J[2][2]);
	void getJacobian(float J[2][2]);
	void FK(float qq0, float qq1, float &r, float &th);
	void IK(float r, float theta, float &q0, float &q1);

	float getOpenLoop(int getAng);
	void setOpenLoop(int getAng, float f);
	
	void setGain(int type, float kp, float kd); 
	void setGain(int type, float kp);
	void setPosition(int getAng, float des);   

	// void setForce(float fx, float fy); 
	// void getForce(float &fx, float &fy);
};

#endif