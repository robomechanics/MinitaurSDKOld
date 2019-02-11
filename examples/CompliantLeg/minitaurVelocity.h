#include <math.h>
#include <stdio.h>
#include <SDK.h>
#include <vector>
#include <Motor.h>
#include <ReorientableBehavior.h>
using namespace std;

#define numMotors 10
#define numSamplesPos 10
#define numSamplesVel 25

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
	float median(float *vec,int start,int length);
public:
	minitaurVelocity();
	void init();
	void updateVelocity();
	void dumpData();
	float filteredVel[numMotors];
};