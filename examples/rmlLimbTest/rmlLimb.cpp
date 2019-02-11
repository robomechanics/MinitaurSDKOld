#include "rmlLimb.h"
#include <stdio.h>
#include <SDK.h>
#include <math.h>
#include <Motor.h>

rmlLimb::rmlLimb()
{
	l0 = 0.1; 
	l1 = 0.2; 
	idx0 = 1; 
	idx1 = 0;

	q0 = 0.f; 
	q1 = 0.f; 
	dq0 = 0.f; 
	dq1 = 0.f; 

	kp = 0.f; 
	kd = 0.f; 
	kpr = 0.f; 
	kdr = 0.f;
	kpt = 0.f; 
	kdt = 0.f; 

	/// joint[i] is initialized by init(RobotParams_Type_MINITAUR, argc, argv); which is called in main
	// Need to:
	// call C->mode = RobotCommand_Mode_JOINT at beginning of behavior update function
	// call P->limbs[0].type = LimbParams_Type_SYMM5BAR_EXT_M or _RAD to use meters or extension angle for leg length;
	// call RMLlimb[i].updateState() at beginning of behavior update function
	// call RMLlimb[i].updateCommand() at end of behavior update function  
}

void rmlLimb::Init(int legnumber)
{
	if (legnumber == 0)
	{
		idx0 = 1;
		idx1 = 0;
	}
	else if (legnumber == 1)
	{
		idx0 = 3;
		idx1 = 2; 
	}
	else if (legnumber == 2)
	{
		idx0 = 4;
		idx1 = 5; 
	}
	else if (legnumber == 3)
	{
		idx0 = 6;
		idx1 = 7; 
	}
	else 
	{
		printf("legnumber must be between [0,3]!!!\n");
	}

}

void rmlLimb::updateState(void)
{
	q0 = joint[idx0].getPosition(); /// get joint pos 
	q1 = joint[idx1].getPosition(); 	
	dq0 = joint[idx0].getVelocity(); /// get joint pos 
	dq1 = joint[idx1].getVelocity(); 	

	usePolar = P->limbs[0].type;
}

void rmlLimb::updateCommand(void)
{
	float J[2][2]; // will save Jacobian here
	float t[2];    // will save needed torque here 

	// get what you need
	getJacobian(q0,q1,J);

	// do transpose
	float tmp = J[1][0]; 
	J[1][0] = J[0][1]; 
	J[0][1] = tmp; 

	// [uq0; uq1] = J'*[ur; uth]   
	t[0] = J[0][0]*ur + J[0][1]*uth; 
	t[1] = J[1][0]*ur + J[1][1]*uth; 

	joint[idx0].setOpenLoop(t[0]); 
	joint[idx1].setOpenLoop(t[1]); 
}

float rmlLimb::getPosition(int getAng)
{
	// float qq0 = 0.f; 
	// float qq1 = 0.f; 
	// float q0 = joint[idx0].getPosition(); /// get joint pos 
	// float q1 = joint[idx1].getPosition(); 	
	float th, r; 

	FK(q0,q1,r,th);	/// Call Forward kinematics to find r and th
	if (getAng == 1)
	{	
		return th; 	
	}
	else
	{
		return r; 	
	}
}

float rmlLimb::getVelocity(int getAng)
{
    float J[2][2];
    getJacobian(q0,q1,J);

    if (getAng == 1)
    {
    	float dth = J[1][0]*dq0 + J[1][1]*dq1;
    	return dth;
    }
    else 
    {
    	float dr = J[0][0]*dq0 + J[0][1]*dq1;
    	return dr; 
    }
}

// getting jacobian from given q0, q1
void rmlLimb::getJacobian(float qq0, float qq1, float J[2][2])
{

	float qmean = (qq0 + qq1)/2.f;
	float qdiff = (qq0 - qq1)/2.f;

	// Force extension to be between -pi and pi
	qmean = fmod(qmean, PI);

	// If extension is negative, encoders think leg is flipped,  so unflip by adding PI to extension and angle and rewrap extension. See MinitaurLeg.cpp for details
	if (qmean < 0.0) {
    	qmean += PI;
    	float f = qdiff + PI;
    	if (f>0)
    		qdiff = (fmodf(f+PI, TWO_PI) - PI);
 		else
    		qdiff = (fmodf(f-PI, TWO_PI) + PI);
  	} 	

	// J = [dr/dq0, dr/dq1; dth/dq0, dth/dq1]
	if(usePolar)
	{
		J[0][0] = (l0*fastsin(qmean))/2.f - (l0*l0*fastcos(qmean)*fastsin(qmean))/
										(2.f*fastsqrt(l1*l1 - l0*l0*fastsin(qmean)*fastsin(qmean)));

		J[0][1] = (l0*fastsin(qmean))/2.f - (l0*l0*fastcos(qmean)*fastsin(qmean))/    
										(2.f*fastsqrt(l1*l1 - l0*l0*fastsin(qmean)*fastsin(qmean)));
	} else {
		J[0][0] = 0.5f;
		J[0][1] = 0.5f;
	}


	J[1][0] = 0.5f; 
	J[1][1] = -0.5f; 
}

// getting jacobian from updated members q0, q1
void rmlLimb::getJacobian(float J[2][2])
{

	float qmean = (q0 + q1)/2.f;
	float qdiff = (q0 - q1)/2.f;

	// Force extension to be between -pi and pi
	qmean = fmod(qmean, PI);

	// If extension is negative, encoders think leg is flipped,  so unflip by adding PI to extension and angle and rewrap extension. See MinitaurLeg.cpp for details
	if (qmean < 0.0) {
    	qmean += PI;
    	float f = qdiff + PI;
    	if (f>0)
    		qdiff = (fmodf(f+PI, TWO_PI) - PI);
 		else
    		qdiff = (fmodf(f-PI, TWO_PI) + PI);
  	} 	

	// J = [dr/dq0, dr/dq1; dth/dq0, dth/dq1]
	if(usePolar)
	{
		J[0][0] = (l0*fastsin(qmean))/2.f - (l0*l0*fastcos(qmean)*fastsin(qmean))/
										(2.f*fastsqrt(l1*l1 - l0*l0*fastsin(qmean)*fastsin(qmean)));

		J[0][1] = (l0*fastsin(qmean))/2.f - (l0*l0*fastcos(qmean)*fastsin(qmean))/    
										(2.f*fastsqrt(l1*l1 - l0*l0*fastsin(qmean)*fastsin(qmean)));
	} else {
		J[0][0] = 0.5f;
		J[0][1] = 0.5f;
	}

	J[1][0] = 0.5f; 
	J[1][1] = -0.5f; 
}

void rmlLimb::FK(float qq0,float qq1, float &r, float &th)
{
	float qmean = (qq0 + qq1)/2.f;
	float qdiff = (qq0 - qq1)/2.f;

	// Force extension to be between -pi and pi
	qmean = fmod(qmean, PI);

	// If extension is negative, encoders think leg is flipped,  so unflip by adding PI to extension and angle and rewrap extension. See MinitaurLeg.cpp for details
	if (qmean < 0.0) {
    	qmean += PI;
    	float f = qdiff + PI;
    	if (f>0)
    		qdiff = (fmodf(f+PI, TWO_PI) - PI);
 		else
    		qdiff = (fmodf(f-PI, TWO_PI) + PI);
  	} 	
	th = qdiff; 

	if (usePolar)
	{
		r = -l0*fastcos(qmean)+fastsqrt(l1*l1-l0*l0*fastsin(qmean)*fastsin(qmean));
	} else {
		r = qmean;
	}
	
}

void rmlLimb::IK(float r,float theta, float &q0, float &q1)
{
	float qmean = 0;
	if (usePolar){
		float rr = r; 
		if (rr < (l1 - l0)) // l1 - l0 = 0.1 
		{
			rr = l1 - l0; 
		}
		if (rr > (l1 + l0)) // l1 + l0 = 0.3 
		{
			rr = l1 + l0; 
		}
		float A = (l0*l0 + rr*rr - l1*l1)/(2.*l0*rr); 
		qmean = PI - acos(A);
	} else {
		qmean = r;
	}
	q0 = qmean + theta;
	q1 = qmean - theta;
}

float rmlLimb::getOpenLoop(int getAng)
{
	if (getAng == 0) // EXTENSION
	{
		return ur; 
	}
	else if (getAng == 1) // ANGLE
	{
		return uth; 
	}
	else
	{
		return ur; 
	}
}

void rmlLimb::setOpenLoop(int getAng, float f)
{

	if (f < -2)
	{
		printf("the force must be between -1 and 1 !!!\n");
		f = -2.f; 
	}
	else if (f > 2)
	{
		printf("the force must be between -1 and 1 !!!\n");
		f = 2.f; 
	}

	if (getAng == 0) // EXTENSION
	{
		ur = f; 
	}	
	else if (getAng == 1) // ANGLE
	{
		uth = f; 
	}
	else 
	{
		// throw error here?
		printf("type must either be 0(EXTENSION) or 1(ANGLE)!!!\n");
		ur = f; 
	}
}

void rmlLimb::setGain(int getAng, float kp, float kd)
{
	if (getAng == 0) // EXTENSION
	{
		kpr = kp; 
		kdr = kd; 
	}
	else if (getAng == 1) // ANGLE
	{
		kpt = kp; 
		kdt = kd; 
	}
	else if (getAng == -1) // JOINT
	{
		this->kp = kp; // UNSURE IF THS HAS BEEN TESTED (but why would anyone use it?)
		this->kd = kd; 
	}
} 

void rmlLimb::setGain( int getAng, float kp){setGain(getAng, kp, 0);}

void rmlLimb::setPosition(int getAng, float des)
{
	// getAng = 0 : EXTENSION
	// getAng = 1 : ANGLE
	float desExtension, desAngle, r, th;
	// current extension: fk(qo,q1, &r, &th)
	// [dr;dtheta] = J*[dq0;dq1]
	FK(q0,q1, r, th);
	float dr = getVelocity(0);
	float dth = getVelocity(1);

	if (getAng == 0) //EXTENSION
	{
		desExtension = des;
		ur = -kpr*(r - desExtension) - kdr*(dr);
	} 
	else if (getAng == 1) //ANGLE
	{
		desAngle = des;
		uth = -kpt*(th - desAngle) - kdt*(dth);
	}
	else
	{
		printf("type must either be 0 or 1!!!\n"); 
	}
}

// void rmlLimb::setForce(float fr, float fth) //, float &t0, float &t1)
// {
// 	float J[2][2]; // will save Jacobian here
// 	float t[2];    // will save needed torque here 
	
// 	// get what you need
// 	getJacobian(q0,q1,J);
	
// 	// do transpose
// 	float tmp = J[1][0]; 
// 	J[1][0] = J[0][1]; 
// 	J[0][1] = tmp; 

// 	// t = J'*F   
// 	t[0] = J[0][0]*fr + J[0][1]*fth; 
// 	t[1] = J[1][0]*fr + J[1][1]*fth; 

// 	// do whatever to set the torque of motor 
// 	// joint[idx0].setOpenLoop(t[0]);
// 	// joint[idx1].setOpenLoop(t[1]);  

// }

// void rmlLimb::getForce(float &fx, float &fy)
// {
// 	float J[2][2]; // will save Jacobian here
// 	float t[2];    // will save needed torque here 

// 	// get what you need
// 	getJacobian(q0,q1,J);

// 	// get applied torque 
// 	t[0] = 0.f;
// 	t[1] = 0.f; 
// 	// joint[idx0].getOpenLoop(t[0]);
// 	// joint[idx1].getOpenLoop(t[1]);  

// 	// do transpose
// 	float tmp = J[1][0]; 
// 	J[1][0] = J[0][1]; 
// 	J[0][1] = tmp; 

// 	// do inverse 
// 	float det; 
// 	float invJ[2][2]; 
// 	det = J[0][0]*J[1][1] - J[0][1]*J[1][0]; 
// 	invJ[0][0] = J[1][1]/det; 
// 	invJ[0][1] = -J[0][1]/det; 
// 	invJ[1][0] = -J[1][0]/det; 
// 	invJ[1][1] = J[0][0]/det; 

// 	// do multiplication F = (J')^-1*T
// 	fx = J[0][0]*t[0] + J[0][1]*t[1]; 
// 	fy = J[1][0]*t[0] + J[1][1]*t[1]; 
// }