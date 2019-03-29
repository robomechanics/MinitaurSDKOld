#include "minitaurVelocity.h"
#define thresh 250
minitaurVelocity::minitaurVelocity()
{
	initMean();
}

void minitaurVelocity::init()
{
	// general variable to init for all methods
	indexPos = 0;
	indexVel = 0;
	posFilled = false;
	velFilled = false;
	lastT = 0;
	free(tRecord);
	free(posRecord);
	free(velRecord);
	tRecCount = 0;
	posRecCount = 0;
	velRecCount = 0;
	for (int i = 0; i<numMotors; i++)
		filteredVel[i] = 0;
}

void minitaurVelocity::initMean(int numSamplesPos_,int numSamplesVel_)
{
	numSamplesPos = numSamplesPos_;
	numSamplesVel = numSamplesVel_;
	init();
	initMean();
}

void minitaurVelocity::initMean()
{
	filterType = 0;
	init();
	for (int i = 0; i < numMotors;i++)
		filteredVel[i] = 0;
	tRecord = (uint32_t*)malloc(numSamplesPos*sizeof(uint32_t));
	posRecord = (float*)malloc(numSamplesPos*numMotors*sizeof(float));
	velRecord = (float*)malloc(numMotors*numSamplesVel*sizeof(float));
}

void minitaurVelocity::initExponential(float expAlpha_)
{
	expAlpha = expAlpha_;
}

void minitaurVelocity::initExponential()
{
	filterType = 1;
	init();
	filterType = 1;
	posRecord = (float*)malloc(numMotors*sizeof(float));
	for (int i = 0; i < numMotors;i++)
		filteredVel[i] = 0;
}

void minitaurVelocity::updateVelocity()
{
	switch(filterType)
	{
		case 0: 
			updateVelocityMean();
			break;
		case 1: 
			updateVelocityExponential();
			break;
	}
}

void minitaurVelocity::updateVelocityExponential()
{
	//Record time and motor position
	if (lastT - clockTimeUS < thresh)
		return;
	else
		lastT = clockTimeUS;

	if (posFilled)
	{
		for (int i = 0; i < numMotors; i++)
		{
			filteredVel[i] = (1-expAlpha)*filteredVel[i] + expAlpha*(joint[i].getPosition()-posRecord[i])/((float)(clockTimeUS-lastT)/1000000.0);
			posRecord[i] = joint[i].getPosition();
		}
	}
	else
	{
		for (int i = 0; i < numMotors; i++)
			posRecord[i] = joint[i].getPosition();
		posFilled = true;
	}
}

void minitaurVelocity::updateVelocityMean()
{
	//Record time and motor position
	if (lastT - clockTimeUS < thresh)
		return;
	else
		lastT = clockTimeUS;
	tRecord[indexPos] = clockTimeUS;
	for (int i = 0; i < numMotors; i++)
		posRecord[indexPos+numSamplesPos*i] = joint[i].getPosition();
	//Check if enough samples of motor positions has been taken to start calculating velocity
	posFilled = posFilled || (indexPos == (numSamplesPos-1));
	//Convert motor position to raw velocity (given frame of size numSamplesPos steps)
	if (posFilled)
	{
		for (int i = 0; i < numMotors; i++)
		{
			float posNext = posRecord[indexPos+numSamplesPos*i];
			float posLast = posRecord[((indexPos+1)%numSamplesPos)+numSamplesPos*i];
			if (posNext > 5.0*PI/6.0)
			{
				if (posLast < -5.0*PI/6.0)
					posLast = posLast + 2.0*PI;
			}
			else if (posNext < -5.0*PI/6.0)
			{
				if (posLast > 5.0*PI/6.0)
					posLast = posLast - 2.0*PI;
			}
			velRecord[indexVel+numSamplesVel*i] = (posNext - posLast)/((float)(tRecord[indexPos]-tRecord[(indexPos+1)%numSamplesPos])/1000000.0);
		}
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