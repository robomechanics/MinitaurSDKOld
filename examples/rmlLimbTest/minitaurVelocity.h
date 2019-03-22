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
#define numSamplesPos 2
#define numSamplesVel 2

class minitaurVelocity
{
public:
	int indexPos = 0;
	int indexVel = 0;
	bool posFilled = false;
	bool velFilled = false;
	uint32_t lastT = 0;
	uint32_t tRecord[numSamplesPos];
	float posRecord[numSamplesPos*numMotors];
	float velRecord[numMotors*numSamplesVel];
	int tRecCount = 0;
	int posRecCount = 0;
	int velRecCount = 0;
	int filtVelCount = 0;
	uint32_t timeLastPrint = 0;
	float median(float *vec,int start,int length);
	float mean(float *vec,int start,int length);
public:
	minitaurVelocity();
	void init();
	void updateVelocity();
	void resetDump();
	bool dumpData();
	float filteredVel[numMotors];
};

#endif