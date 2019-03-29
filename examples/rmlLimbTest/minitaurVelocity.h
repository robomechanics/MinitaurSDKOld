#ifndef MINITAURVELOCITY_H
#define MINITAURVELOCITY_H

#include <math.h>
#include <stdio.h>
#include <SDK.h>
#include <vector>
#include <Motor.h>
#include <ReorientableBehavior.h>
using namespace std;

#define tWait 50000
#define numMotors 10

class minitaurVelocity
{
private:
	int numSamplesPos = 2;
	int numSamplesVel = 2;
	int indexPos = 0;
	int indexVel = 0;
	bool posFilled = false;
	bool velFilled = false;
	
	int tRecCount = 0;
	int posRecCount = 0;
	int velRecCount = 0;
	float mean(float *vec,int start,int length);
	void updateVelocityMean();
	void updateVelocityExponential();
	int filterType = 0; //0 for mean filter, 1 for exponential filter
	float expAlpha;

	// variable used for outputting data
	int filtVelCount = 0;
	uint32_t timeLastPrint = 0;
public:
	uint32_t lastT = 0;
	uint32_t* tRecord;
	float *posRecord;
	float *velRecord;
	minitaurVelocity();
	void init();
	void initMean(); // initialize mean filter
	void initMean(int numSamplesPos_,int numSamplesVel_);
	void initExponential(); // initialize exponential filter
	void initExponential(float expAlpha_);
	void updateVelocity();
	void resetDump(); //for troubleshooting ignore
	bool dumpData(); // for troubleshooting ignore
	float filteredVel[numMotors];// use this to access filtered velocities
};

#endif