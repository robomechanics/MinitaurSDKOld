#include "minitaurVelocity.h"
#define thresh 250
minitaurVelocity::minitaurVelocity(){}

void minitaurVelocity::init()
{
}

/*
void minitaurVelocity::init(int _numMotors)
{
	numMotors = _numMotors;
	initVectors();
}

void minitaurVelocity::init(int _numMotors,int _numSamplesPos, int _numSamplesVel)
{
	numMotors = _numMotors;
	numSamplesPos = _numSamplesPos;
	numSamplesVel = _numSamplesVel;
	initVectors();
}


void minitaurVelocity::initVectors()
{
	posRecord.clear();
	velRecord.clear();
	filteredVel.clear();
	tRecord.clear();
	for (int i = 0;i < numMotors*numSamplesPos;i++)
		posRecord.push_back(0);
	for (int i = 0; i < numMotors*numSamplesVel;i++)
		velRecord.push_back(0);
	for (int i = 0; i < numMotors;i++)
		filteredVel.push_back(0);
	for (int i = 0;i < numSamplesPos;i++)
		tRecord.push_back(0);
}
*/

void minitaurVelocity::updateVelocity()
{
	//Record time and motor position
	if (lastT - clockTimeUS < thresh)
		return;
	tRecord[indexPos] = clockTimeUS;
	for (int i = 0; i < numMotors; i++)
		posRecord[indexPos+numSamplesPos*i] = joint[i].getPosition();
	//Check if enough samples of motor positions has been taken to start calculating velocity
	posFilled = posFilled || (indexPos == (numSamplesPos-1));
	//Convert motor position to raw velocity (given frame of size numSamplesPos steps)
	if (posFilled)
	{
		for (int i = 0; i < numMotors; i++)
			velRecord[indexVel+numSamplesVel*i] = (posRecord[indexPos+numSamplesPos*i] - posRecord[((indexPos+1)%numSamplesPos)+numSamplesPos*i])/((tRecord[indexPos]-tRecord[(indexPos+1)%numSamplesPos])/1000000.0);
	}

	//Check if enough velocity calculations has been done to start filtering
	velFilled = velFilled || (indexVel == (numSamplesVel-1));
	if (velFilled)
	{
		for (int i = 0; i < numMotors; i++)
			filteredVel[i] = mean(velRecord,numSamplesVel*i,numSamplesVel);
	}
	indexPos++;
	indexPos = indexPos%numSamplesPos;
	if (posFilled)
	{
		indexVel++;
		indexVel = indexVel%numSamplesVel;
	}
}

float minitaurVelocity::median(float *vec,int start,int length)
{
	//First copy vec to sorted while sorting the data
	float sorted[length];
	for (int i = 0;i < length;i++)
	{
		int index=0;
		for (int j = 0; j < i+1;j++)
		{
			index = j;
			if (vec[i+start] < sorted[j])
				break;
		}
		for (int k = length;k > index;k--)
			sorted[k] = sorted[k-1];
		sorted[index] = vec[i+start];
	}
	if (length%2)
	{
		return sorted[(int)floor(length/2)];
	}
	else
		return (sorted[length/2]+sorted[length/2-1])/2;
}

float minitaurVelocity::mean(float *vec,int start,int length)
{
	float sum = 0;
	for (int i = 0; i < length; i++)
	{
		sum = sum+vec[i+start];
	}
	return sum/(float)length;
}

void minitaurVelocity::resetDump()
{
	tRecCount = 0;
	posRecCount = 0;
	velRecCount = 0;
	filtVelCount = 0;
}

bool minitaurVelocity::dumpData()
{
	if (tRecCount < numSamplesPos)
	{
		if (tRecCount==0)
			printf("tRecord =  [\n");
		if (clockTimeUS-timeLastPrint > tWait)
		{
			timeLastPrint = clockTimeUS;
			printf("%lu, \n", tRecord[tRecCount]);
			tRecCount++;
		}
		if (tRecCount == numSamplesPos)
			printf("] \n");
	}
	else if (posRecCount < numSamplesPos*numMotors)
	{
		if ((clockTimeUS-timeLastPrint) > tWait)
		{
			timeLastPrint = clockTimeUS;
			if (posRecCount == 0)
				printf("posRecord = [\n");
			printf("%f, \n", posRecord[posRecCount]);
			posRecCount++;
		}
		if (posRecCount == numSamplesPos*numMotors)
			printf("] \n");
	}
	else if (velRecCount < numSamplesVel*numMotors)
	{
		if (clockTimeUS-timeLastPrint > tWait)
		{
			timeLastPrint = clockTimeUS;
			if (velRecCount == 0)
				printf("velRecord = [\n");
			printf("%f,\n", velRecord[velRecCount]);
			velRecCount++;
		}
		if (velRecCount == numSamplesVel*numMotors)
			printf("] \n");
	}
	else if (filtVelCount < numMotors)
	{
		if (clockTimeUS-timeLastPrint > tWait)
		{
			timeLastPrint = clockTimeUS;
			if (filtVelCount == 0)
				printf("filteredVel = [\n");
			printf("%f,\n", filteredVel[filtVelCount]);
			filtVelCount++;
		}
		if (filtVelCount == numSamplesVel*numMotors)
			printf("] \n");
	}
	else
	{
		return true;
	}
	return false;
}