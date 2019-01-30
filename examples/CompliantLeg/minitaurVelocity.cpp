#include "minitaurVelocity.h"

minitaurVelocity::minitaurVelocity(){}

void minitaurVelocity::init()
{
	initVectors();
}

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

void minitaurVelocity::updateVelocity()
{
	//Record time and motor position
	tRecord[indexPos] = clockTimeUS;
	for (int i = 0; i < numMotors; i++)
		posRecord[indexPos+numMotors*i] = joint[i].getPosition();

	//Check if enough samples of motor positions has been taken to start calculating velocity
	posFilled = posFilled || (indexPos == (numSamplesPos-1));
	//Convert motor position to raw velocity (given frame of size numSamplesPos steps)
	if (posFilled)
	{
		for (int i = 0; i < numMotors; i++)
			velRecord[indexVel+numMotors*i] = (posRecord[indexPos+numMotors*i] - posRecord[((indexPos+1)%numSamplesPos)+numMotors*i])/((tRecord[indexPos]-tRecord[(indexPos+1)%numSamplesPos])/1000000.0);
	}

	//Check if enough velocity calculations has been done to start filtering
	velFilled = velFilled || (indexVel == (numSamplesVel-1));
	if (velFilled)
	{
		for (int i = 0; i < numMotors; i++)
			filteredVel[i] = median(&velRecord,numMotors*i,numSamplesVel);
	}
	indexPos++;
	indexPos = indexPos%numSamplesPos;
	indexVel++;
	indexVel = indexVel%numSamplesVel;
}

float minitaurVelocity::median(vector<float> *vec,int start,int length)
{
	//First copy vec to sorted while sorting the data
	float sorted[length];
	for (int i = 0;i < length;i++)
	{
		int index=0;
		for (int j = 0; j < i+1;j++)
		{
			index = j;
			if ((*vec)[i+start] < sorted[j])
				break;
		}
		for (int k = length;k > index;k--)
			sorted[k] = sorted[k-1];
		sorted[index] = (*vec)[i+start];
	}
	if (length%2)
	{
		return sorted[(int)floor(length/2)];
	}
	else
		return (sorted[length/2]+sorted[length/2-1])/2;
}