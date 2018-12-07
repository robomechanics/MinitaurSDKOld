#ifndef RMLLIMB_H
#define RMLLIMB_H

// #include <stdio.h>
// #include <SDK.h>
// #include <math.h>
// #include <Motor.h>

class rmlLimb
{
public:
	float l0, l1; 
	int idx0, idx1; 
	float q0, q1, dq0, dq1; 

	rmlLimb(); 
	void Init(int legnumber); 
	float getPos(int getAng); 
	void getJacobian(float q0, float q1, float J[2][2]);
	void setOpenLoop(float fr, float fth);
	void setForce(float fx, float fy); 
	void getForce(float &fx, float &fy);
	void Fk(float q0, float q1, float &r, float &th);
	void Ik(float r, float theta, float &q0, float &q1);
	void updateStates(void);
	void updateCommand(void); 
};

#endif