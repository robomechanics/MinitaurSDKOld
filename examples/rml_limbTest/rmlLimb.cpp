#include "rmlLimb.h"

#include <stdio.h>
#include <SDK.h>
#include <math.h>
#include <Motor.h>

rmlLimb::rmlLimb()	/// Should we & C inthe constructor so we can set mode?
{
	l0 = 0.1; 
	l1 = 0.2; 
	idx0 = 1; 
	idx1 = 0;

	q0 = 0.f; 
	q1 = 0.f; 

	/// joint[i] is initialized by init(RobotParams_Type_MINITAUR, argc, argv); which is called in main
	// C->mode = RobotCommand_Mode_JOINT; 
}

void rmlLimb::Init(int legnumber)
{
	if (legnumber == 0)
	{
		// idx0 = 1;
		// idx1 = 0;
	}
	else if (legnumber == 1)
	{
		// idx0 = 3;
		// idx1 = 2; 
	}
	else if (legnumber == 2)
	{
		// idx0 = 4;
		// idx1 = 5; 
	}
	else if (legnumber == 3)
	{
		// idx0 = 6;
		// idx1 = 7; 
	}
	else
	{

	}

}

float rmlLimb::getPos(int getAng)
{
	float q0 = 0.f; 
	float q1 = 0.f; 
	// float q0 = joint[idx0].getPosition(); /// get joint pos 
	// float q1 = joint[idx1].getPosition(); 	
	float th, r; 

	Fk(q0,q1,r,th);	/// Call Forward kinematics to find r and th
	if (getAng == 1)
	{	
		return th; 	
	}
	else
	{
		return r; 	
	}
}

void rmlLimb::getJacobian(float q0, float q1, float J[2][2])
{
	// J = [dr/dq0, dr/dq1; dth/dq0, dth/dq1]
	J[0][0] = (l0*sin(q0/2.f + q1/2.f))/2.f - (l0*l0*cos(q0/2.f + q1/2.f)*sin(q0/2.f + q1/2.f))/
											(2.f*sqrt(l1*l1 - l0*l0*sin(q0/2.f+q1/2.f)*sin(q0/2.f+q1/2.f)));

	J[0][1] = (l0*sin(q0/2.f + q1/2.f))/2.f - (l0*l0*cos(q0/2.f + q1/2.f)*sin(q0/2.f + q1/2.f))/
											(2.f*sqrt(l1*l1 - l0*l0*sin(q0/2.f+q1/2.f)*sin(q0/2.f+q1/2.f)));

	J[1][0] = 0.5f; 
	J[1][1] = -0.5f; 
}

void rmlLimb::setOpenLoop(float fr, float fth)
{
	float J[2][2]; // will save Jacobian here
	float t[2];    // will save needed torque here 
	
	// get what you need
	float q0 = 0.f; 
	float q1 = 0.f;  
	// float q0 = joint[idx0].getPosition(); // get joint pos 
	// float q1 = joint[idx1].getPosition(); 
	getJacobian(q0,q1,J);
	
	// do transpose
	float tmp = J[1][0]; 
	J[1][0] = J[0][1]; 
	J[0][1] = tmp; 

	// t = J'*F   
	t[0] = J[0][0]*fr + J[0][1]*fth; 
	t[1] = J[1][0]*fr + J[1][1]*fth; 

	// do whatever to set the torque of motor 
	// joint[idx0].setOpenLoop(t[0]);
	// joint[idx1].setOpenLoop(t[1]);
	// return 1.f;
}

void rmlLimb::setForce(float fr, float fth) //, float &t0, float &t1)
{
	float J[2][2]; // will save Jacobian here
	float t[2];    // will save needed torque here 
	
	// get what you need
	float q0 = 0.f; 
	float q1 = 0.f;  
	// float q0 = joint[idx0].getPosition(); // get joint pos 
	// float q1 = joint[idx1].getPosition(); 
	getJacobian(q0,q1,J);
	
	// do transpose
	float tmp = J[1][0]; 
	J[1][0] = J[0][1]; 
	J[0][1] = tmp; 

	// t = J'*F   
	t[0] = J[0][0]*fr + J[0][1]*fth; 
	t[1] = J[1][0]*fr + J[1][1]*fth; 

	// do whatever to set the torque of motor 
	// joint[idx0].setOpenLoop(t[0]);
	// joint[idx1].setOpenLoop(t[1]);  

}

void rmlLimb::getForce(float &fx, float &fy)
{
	float J[2][2]; // will save Jacobian here
	float t[2];    // will save needed torque here 

	// get what you need
	float q0 = 0.f; 
	float q1 = 0.f; 
	// float q0 = joint[idx0].getPosition(); // get joint pos 
	// float q1 = joint[idx1].getPosition(); 
	getJacobian(q0,q1,J);

	// get applied torque 
	t[0] = 0.f;
	t[1] = 0.f; 
	// joint[idx0].getOpenLoop(t[0]);
	// joint[idx1].getOpenLoop(t[1]);  

	// do transpose
	float tmp = J[1][0]; 
	J[1][0] = J[0][1]; 
	J[0][1] = tmp; 

	// do inverse 
	float det; 
	float invJ[2][2]; 
	det = J[0][0]*J[1][1] - J[0][1]*J[1][0]; 
	invJ[0][0] = J[1][1]/det; 
	invJ[0][1] = -J[0][1]/det; 
	invJ[1][0] = -J[1][0]/det; 
	invJ[1][1] = J[0][0]/det; 

	// do multiplication F = (J')^-1*T
	fx = J[0][0]*t[0] + J[0][1]*t[1]; 
	fy = J[1][0]*t[0] + J[1][1]*t[1]; 
}

void rmlLimb::Fk(const float q0,const float q1, float &r, float &th)
{
	float qavg = (q0 - q1)/2.f;
	float qdiff = (q0 + q1)/2.f;
	th = qavg;
	r = -l0*cos(qdiff)+sqrt(l1*l1-l0*l0*sin(qdiff));
}

void rmlLimb::Ik(const float r,const float theta, float &q0, float &q1)
{
	float rr = r; 
	if (rr < l1 - l0) // l1 - l0 = 0.1 
	{
		rr = l1 - l0; 
	}
	if (rr > l1 + l0) // l1 + l0 = 0.3 
	{
		rr = l1 + l0; 
	}
	float A = (l0*l0 + rr*rr - l1*l1)/(2.*l0*rr); 
	// clipping A between -1 and 1 so that 
	// we don't get a nan for acos(A) 
	if (A > 1.)
	{
		A = 1.f; 
	}
	if (A < -1.)
	{
		A = -1.f; 
	}
	float qdiff = PI - acos(A);
	q0 = qdiff + theta;
	q1 = qdiff -theta;
}

// void rmlLimb::

// int main(void)
// {
// // rmlLimb limbs[4]; 
// // limbs[0].Init(0); 
// // limbs[1].Init(1); 

// 	float J[2][2]; // jacobian 
// 	float fx, fy; // end effector force
// 	rmlLimb limb; 


// 	// ========= JACOBIAN =============
// 	float q0 = 0.f; 
// 	float q1 = 0.f; 
// 	// float q0 = joint[idx0].getPosition(); // get joint pos 
// 	// float q1 = joint[idx1].getPosition(); 
// 	limb.getJacobian(q0,q1,J); 
// 	printf("%lf %lf\n%lf %lf\n",J[0][0],J[0][1],J[1][0],J[1][1]);


// 	// ======== SET FORCE ========== 
// 	fx = 0.f; // set our desired force
// 	fy = -10.f; 
// 	limb.setForce(fx, fy); 
// 	printf("desired force\nfx:%lf fy:%lf\n",fx,fy);


// 	// ======== GET FORCE ==========
// 	limb.getForce(fx, fy); 
// 	printf("force applied\nfx:%lf fy:%lf\n",fx,fy);	

// 	// ======== KINEMATICS =========
// 	float r, th; 
// 	q0 = 0.; 
// 	q1 = PI; 
// 	limb.Fk(q0,q1,r,th);
// 	printf("given q0:%lf q1:%lf forward kinematics r:%lf th:%lf\n",q0,q1,r,th); 

// 	limb.Ik(r,th,q0,q1); 
// 	printf("given  r:%lf th:%lf inverse kinematics q0:%lf q1:%lf\n",r,th,q0,q1);

// 	// ========== GET POS ==========
// 	int getAng = 1; 
// 	th = limb.getPos(getAng); 
// 	printf("pos th:%lf\n",q0,q1);

// 	return 1; 
// }